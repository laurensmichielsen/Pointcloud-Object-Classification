import rclpy

import numpy as np
import time
from rclpy.node import Node

from pc_processing.filter import Filter
from pc_processing.ground_removal import RansacGroundRemoval
from pc_processing.pc_clustering import PointCloudDBSCAN
from pc_processing.reconstruct import ReconstructCone
from pc_processing.utils import to_pointcloud_msg, parse_pointcloud_msg

from geometry_msgs.msg import Point
from labeling_msgs.msg import Cluster
from labeling_msgs.srv import LabelingAssistRequest

from .detector import Detector

class LabelingAssistNode(Node):
    def __init__(self):
        super().__init__("labeling_assist")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("lidar_frame_id", "rslidar"),
                ("input_pointcloud_topic", "/rslidar_points"),
                ("ground_removal.number_of_iterations", 30),
                ("ground_removal.number_of_initial_seeds", 10),
                ("ground_removal.number_of_x_chunks", 10),
                ("ground_removal.number_of_y_chunks", 10),
                ("ground_removal.number_of_points_per_plane", 3),
                ("ground_removal.distance_threshold", 0.02), # distance to the plane
                ("ground_removal.min_points_per_chunk", 50), # min points required to process
                ("ground_removal.remove_min_points", 15), # number of points required to not remove the segment
                ("filter.vehicle_min_x", 0.0),
                ("filter.vehicle_max_x", 2.9),
                ("filter.vehicle_min_y", -1.0),
                ("filter.vehicle_max_y", 1.0),
                ("filter.cone_height", 0.505),
                ("filter.voxel_size", [0.1, 0.1, 0.1]),
                ("binning.max_points_in_bin", 900),
                ("cluster.distance_threshold", .2),
                ("cluster.min_samples", 9),
                ("cluster.max_range", 15.0),
                ("cluster.min_range", 0.5),
                ("reconstruct.width_threshold", 0.1475),
                ("reconstruct.max_height_threshold", 0.45),
                ("reconstruct.min_height_threshold", 0.05),
                ("reconstruct.safety_factor", 1.15),
                ("reconstruct.include_base", False),
                ("reconstruct.base_buffer", 0.05),
                ("cv_viz_flag", True),
            ]
        )

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
            remove_min_points=self.get_parameter("ground_removal.remove_min_points").value
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

        self.detector = Detector(self.filter, self.ground_removal, self.cluster, self.reconstruct)

        self._frame_request_service = self.create_service(
            LabelingAssistRequest, "labeling_assist_request", self.labeling_assist_request_callback
        )

        print("WERE HERE")

    def labeling_assist_request_callback(
        self, request: LabelingAssistRequest.Request, response: LabelingAssistRequest.Response
    ) -> LabelingAssistRequest.Response:
        print("Callback recieved")
        points_2darray = parse_pointcloud_msg(request.pointcloud)

        if points_2darray.shape[1] > 4:
            points_2darray = points_2darray[:, :4]

        clusters = []
        filtered_points = None

        # Forgive me father, for I have sinned. But it is labeling util, so who is ever going to look at this anyways...
        # TODO: PLEASE CONVERT THIS TO LAMBDA FUNCTIONS
        def on_filtered_pointcloud(points, timestamp=None):
            nonlocal filtered_points
            filtered_points = points

        # Did it again, whoops
        def on_clustered_points(unfiltered_clusters, timestamp=None):
            nonlocal clusters
            clusters = unfiltered_clusters

        a_Time = time.time()
        self.detector.process(points_2darray, on_clustered_points, on_filtered_pointcloud, request.pointcloud.header.frame_id, request.pointcloud.header.stamp)
        b_time = time.time() - a_Time

        self.get_logger().info(f"{b_time} time")

        response.clusters = clusters
        response.filtered_pointcloud = filtered_points
        return response


def main():
    rclpy.init()

    labeling_assist_node = LabelingAssistNode()
    rclpy.spin(labeling_assist_node)

    labeling_assist_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
