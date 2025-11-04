import numpy as np
import rclpy

# rclpy helpers
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# ROS messages used in this module
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Point
from builtin_interfaces.msg import Duration
from builtin_interfaces.msg import Time as RosTime

# local utilities / modules (ensure these modules export these names)
from .utils import parse_pointcloud_msg, to_pointcloud_msg
from .filter import Filter
from .ground_removal import RansacGroundRemoval
from .pc_clustering import PointCloudDBSCAN
from .reconstruct import ReconstructCone


class PreProcessingNode(Node):
    """
    This is the pre-processing node which is responsible for converting a point cloud
    into centroids and their associated points
    """
    def __init__(self, full_reliability=False):
        super().__init__("preprocessing")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("lidar_frame_id", "rslidar"),
                ("input_pointcloud_topic", "/rslidar_points"),
                ("ground_removal.number_of_iterations", 50),
                ("ground_removal.number_of_initial_seeds", 10),
                ("ground_removal.number_of_x_chunks", 10),
                ("ground_removal.number_of_y_chunks", 10),
                ("ground_removal.number_of_points_per_plane", 3),
                ("ground_removal.distance_threshold", 0.02), # distance to the plane
                ("ground_removal.min_points_per_chunk", 50),
                ("filter.vehicle_min_x", 0.0),
                ("filter.vehicle_max_x", 2.9),
                ("filter.vehicle_min_y", -1.0),
                ("filter.vehicle_max_y", 1.0),
                ("filter.cone_height", 0.505),
                ("filter.voxel_size", [0.1, 0.1, 0.1]),
                ("binning.max_points_in_bin", 900),
                ("cluster.distance_threshold", .25),
                ("cluster.min_samples", 15),
                ("cluster.max_range", 15.0),
                ("cluster.min_range", 0.5),
                ("reconstruct.width_threshold", 0.1475),
                ("reconstruct.max_height_threshold", 0.45),
                ("reconstruct.min_height_threshold", 0.05),
                ("reconstruct.safety_factor", 1.05),
                ("reconstruct.include_base", False),
                ("reconstruct.base_buffer", 0.05),
                ("cv_viz_flag", True),
            ]
        )

        # Setup the filter
        self.filter = Filter(
            vehicle_min_x=self.get_parameter("filter.vehicle_min_x").value,
            vehicle_max_x=self.get_parameter("filter.vehicle_max_x").value,
            vehicle_min_y=self.get_parameter("filter.vehicle_min_y").value,
            vehicle_max_y=self.get_parameter("filter.vehicle_max_y").value,
            cone_height=self.get_parameter("filter.cone_height").value,
            voxel_size=self.get_parameter("filter.voxel_size").value
        )
        self.debug_mode = self.get_parameter("cv_viz_flag").value

        # Initialize the RANSAC ground removal object
        self.ground_removal = RansacGroundRemoval(
            number_of_iterations=self.get_parameter("ground_removal.number_of_iterations").value,
            number_of_initial_seeds=self.get_parameter("ground_removal.number_of_initial_seeds").value,
            number_of_x_chunks=self.get_parameter("ground_removal.number_of_x_chunks").value,
            number_of_y_chunks=self.get_parameter("ground_removal.number_of_y_chunks").value,
            distance_threshold=self.get_parameter("ground_removal.distance_threshold").value,
            number_of_points_per_plane=self.get_parameter("ground_removal.number_of_points_per_plane").value,
            min_points_per_chunk=self.get_parameter("ground_removal.min_points_per_chunk").value,
        )
        # Setup the clusterer
        self.cluster = PointCloudDBSCAN(
            max_points_in_bin=self.get_parameter("binning.max_points_in_bin").value,
            cluster_distance_threshold=self.get_parameter("cluster.distance_threshold").value,
            cluster_min_samples=self.get_parameter("cluster.min_samples").value,
            cluster_max_range=self.get_parameter("cluster.max_range").value,
            cluster_min_range=self.get_parameter("cluster.min_range").value
        )
        # Setup the cone reconstructor
        self.reconstruct = ReconstructCone(
            width_threshold=self.get_parameter("reconstruct.width_threshold").value,
            max_height_threshold=self.get_parameter("reconstruct.max_height_threshold").value,
            min_height_threshold=self.get_parameter("reconstruct.min_height_threshold").value,
            safety_factor=self.get_parameter("reconstruct.safety_factor").value,
            include_base=self.get_parameter("reconstruct.include_base").value,
            base_buffer=self.get_parameter("reconstruct.base_buffer").value,
        )

        point_cloud_topic = self.get_parameter("input_pointcloud_topic").value
        # setup subscriber for LiDAR data
        if full_reliability:
            full_reliability_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
                depth=200,
                durability=QoSDurabilityPolicy.VOLATILE
            )

            self._subscriber = self.create_subscription(
                PointCloud2, point_cloud_topic, self.on_pointcloud_update, full_reliability_qos
            )
        else:
            self._subscriber = self.create_subscription(
                PointCloud2, point_cloud_topic, self.on_pointcloud_update, 1
            )
        
        # setup publishers for debugging
        self.ground_plane_publisher = self.create_publisher(
            PointCloud2, "/viz/project/pointcloud/ground_removed", 1
        )
        self.filter_publisher = self.create_publisher(
            PointCloud2, "/viz/project/pointcloud/filtered", 1
        )
        self.centroid_marker_publisher = self.create_publisher(
            Marker, "/viz/project/pointcloud/centroids", 1
        )
        self.cone_reconstructed_publisher = self.create_publisher(
            PointCloud2, "/viz/project/pointcloud/cone_proposals", 1
        ) # TODO: make List of Pointclouds, for now just publish just a pc

    
    def on_pointcloud_update(self, raw_pointcloud: PointCloud2):
        # Step 1: convert the PC to a numpy 2D array
        points_2darray = parse_pointcloud_msg(raw_pointcloud)
        print(f"Shape of the input array: {points_2darray}")
        self.points_2darray = points_2darray.copy()
        # check if we only have a 4D array, x, y, z, intensity
        if points_2darray.shape[1] > 4:
            points_2darray = points_2darray[:, :4]
        
        # Step 2: Perform vehicle filtering -> all points that are from the car need to be removed
        points_2darray = self.filter.vehicle_filter(points_2darray)

        # Step 3: Air filtering
        points_2darray = self.filter.air_filter(points_2darray)

        # If debug mode is publish the filtered point cloud
        if self.debug_mode:
            self.visualize_filtered_points(points_2darray, raw_pointcloud.header)
        # Step 4: ground removal
        non_ground_points, ground_points, plane_models = self.ground_removal.remove_ground(points_2darray)

        print(f"The number of non-ground points: {len(non_ground_points)}")
        print(f"The shape of the non-ground points: {non_ground_points.shape}")

        # If debug mode is enabled, publish the filtered pc after ground_removal
        if self.debug_mode:
            self.visualize_ground_removed(non_ground_points, raw_pointcloud.header)
        
        # Step 5: cluster
        centroids = self.cluster.cluster(non_ground_points)

        # if debug mode is enabled, publish the centroids for viz
        if self.debug_mode:
            self.visualize_centroids(centroids, raw_pointcloud.header)
        # Step 6: reconstruct cones
        points_in_boxes = self.reconstruct.reconstruct(self.points_2darray, centroids)
        
        print(points_in_boxes.shape)
        self.visualize_output(points_in_boxes)

    def visualize_filtered_points(self, points, timestamp=None):
        timestamp = self.get_ros_time()
        msg = to_pointcloud_msg(points, frame_id=self.get_parameter("lidar_frame_id").value, timestamp=timestamp)
        self.filter_publisher.publish(msg)

    def visualize_ground_removed(self, points, timestamp=None):
        timestamp=self.get_ros_time()
        msg = to_pointcloud_msg(points, frame_id=self.get_parameter("lidar_frame_id").value, timestamp=timestamp)
        self.ground_plane_publisher.publish(msg)

    def visualize_centroids(self, clustered_points, timestamp=None):
        timestamp = self.get_ros_time()
        marker_points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in clustered_points]
        self.centroid_marker_publisher.publish(self.spheres(marker_points, timestamp=timestamp))

    def spheres(
        self, marker_points, color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0), timestamp=None
    ):
        print(f"Number of centers: {len(marker_points)}")
        timestamp = self.get_ros_time()
        marker = Marker(
            header=Header(frame_id=self.get_parameter("lidar_frame_id").value, stamp=timestamp),
            id=0,
            type=Marker.SPHERE_LIST,
            points=marker_points,
            action=Marker.ADD,
            scale=Vector3(x=0.5, y=0.5, z=0.5),
            color=color,
        )

        return marker

    def visualize_output(self, points, timestamp=None):
        timestamp=self.get_ros_time()
        msg = to_pointcloud_msg(points, frame_id=self.get_parameter("lidar_frame_id").value, timestamp=timestamp)
        self.cone_reconstructed_publisher.publish(msg)

    
    def get_ros_time(self):
        now = self.get_clock().now()
        return RosTime(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])

def main():
    rclpy.init()

    pre_process = PreProcessingNode()
    rclpy.spin(pre_process)

    pre_process.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()