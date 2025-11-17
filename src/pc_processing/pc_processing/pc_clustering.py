import numpy as np
from sklearn.cluster import DBSCAN


class PointCloudClustering:
    """
    Abstract base class for point cloud clustering algorithms.
    """

    def __init__(
        self,
        max_points_in_bin: int,
        cluster_distance_threshold: float,
        cluster_min_samples: int,
        cluster_max_range: float,
        cluster_min_range: float
    ):
        """
        Initialize clustering parameters.
        """
        self.max_points_in_bin = max_points_in_bin
        self.cluster_distance_threshold = cluster_distance_threshold
        self.cluster_max_range = cluster_max_range
        self.cluster_min_range = cluster_min_range
        self.cluster_min_samples = cluster_min_samples

    def _run_clustering(self, points: np.ndarray) -> np.ndarray:
        raise NotImplementedError("Implement this in a subclass")

    def cluster(self, points: np.ndarray) -> np.ndarray:
        """
        Cluster the given points and return cluster centers.
        Args:
            points (np.ndarray): The points as [[x,y,z,i], ...]
        Returns:
            np.ndarray: Cluster centers as [[x, y, z], ...]
        """
        if points.size == 0:
            return np.empty((0, 4))
        
        # Filter by binning (optional noise suppression)
        # binned_points = self.bin_points(points)
        binned_points = points
        print(f"Number of binned points: {len(binned_points)}")
        if binned_points.size == 0:
            return np.empty((0, 4))

        # Run clustering in XY space
        cluster_labels = self._run_clustering(binned_points[:, :2])
        clustered_points = np.column_stack((binned_points, cluster_labels))

        centers = []
        for label in np.unique(cluster_labels):
            if label == -1:  # DBSCAN noise
                continue
            cluster_points = clustered_points[clustered_points[:, -1] == label][:, :3]
            centers.append(np.mean(cluster_points, axis=0))

        return np.array(centers)


class PointCloudDBSCAN(PointCloudClustering):
    """
    Cluster points using DBSCAN.
    """

    def _run_clustering(self, points: np.ndarray) -> np.ndarray:
        clusterer = DBSCAN(
            eps=self.cluster_distance_threshold,
            min_samples=self.cluster_min_samples,
            n_jobs=8
        )
        return clusterer.fit(points).labels_
