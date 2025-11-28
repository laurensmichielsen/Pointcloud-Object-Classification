from labeling_msgs.msg import Cluster
from sensor_msgs.msg import PointCloud2
import numpy as np
from pc_processing.utils import to_pointcloud_msg, parse_pointcloud_msg

class Detector:

    def __init__(self, filter, ground_removal, clusterer, reconstruct):
        self.filter = filter
        self.ground_removal = ground_removal
        self.cluster = clusterer
        self.reconstruct = reconstruct
        self.original_points = None
        print("Init detector done")

    
    def process(self, points_2darray, on_processed_clustered_points, on_processed_reconstructed, frame_id, stamp):
        
        if points_2darray.shape[1] > 4:
            points_2darray = points_2darray[:, :4]

        self.original_points = points_2darray.copy()

        # filtering
        points_2darray = self.filter.vehicle_filter(points_2darray)
        points_2darray = self.filter.air_filter(points_2darray)

        # ground removal
        non_ground_points, groun_points, _ = self.ground_removal.remove_ground(points_2darray)

        centroids = self.cluster.cluster(non_ground_points)

        new_centroids, points_in_boxes = self.reconstruct.reconstruct_labeling(self.original_points, centroids)

        clusters = []
        id_cluster = 0
        all_points = np.array([])
        # make the cluster messages
        for (centroid, points) in zip(new_centroids, points_in_boxes):
            print(centroid)
            cluster = Cluster(
                pointcloud=to_pointcloud_msg(points, frame_id, stamp),
                centroid=centroid,
                type=0,
                id=id_cluster
            )
            id_cluster += 1
            clusters.append(cluster)
            np.append(all_points, points)

        on_processed_clustered_points(clusters)
        # make the full pc of the reconstructed 
        filtered_pc = to_pointcloud_msg(all_points, frame_id, stamp)
        on_processed_reconstructed(filtered_pc)