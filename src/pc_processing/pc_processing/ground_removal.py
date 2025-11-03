import numpy as np
import open3d as o3d

class GroundRemoval:
    """
    Abstract base class to remove ground from a point cloud.
    """
    def __init__(
        self,
        number_of_iterations,
        number_of_initial_seeds,
        number_of_x_chunks,
        number_of_y_chunks,
        distance_threshold,
        number_of_points_per_plane,
        max_distance,
        min_points_per_chunk
    ):
        self.number_of_iterations = number_of_iterations
        self.number_of_initial_seeds = number_of_initial_seeds
        self.number_of_x_chunks = number_of_x_chunks
        self.number_of_y_chunks = number_of_y_chunks
        self.distance_threshold = distance_threshold
        self.number_of_points_per_plane = number_of_points_per_plane
        self.max_distance = max_distance
        self.min_points_per_chunk = min_points_per_chunk

    def remove_ground(self, points_2darray):
        """Removes the ground from a point cloud (to be implemented by subclasses)."""
        raise NotImplementedError("Implement this method in a subclass.")


class RansacGroundRemoval(GroundRemoval):
    """
    Ground removal using RANSAC-based plane segmentation per (x, y) chunk.
    """
    def remove_ground(self, points_2darray):
        points = points_2darray  # alias for readability

        # Compute grid boundaries
        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])

        x_edges = np.linspace(x_min, x_max, self.number_of_x_chunks + 1)
        y_edges = np.linspace(y_min, y_max, self.number_of_y_chunks + 1)

        non_ground_all = []
        ground_all = []
        plane_models = []

        # Iterate over grid chunks
        for i in range(self.number_of_x_chunks):
            for j in range(self.number_of_y_chunks):
                x_mask = (points[:, 0] >= x_edges[i]) & (points[:, 0] < x_edges[i + 1])
                y_mask = (points[:, 1] >= y_edges[j]) & (points[:, 1] < y_edges[j + 1])
                mask = x_mask & y_mask

                chunk_points = points[mask]
                if chunk_points.shape[0] < self.min_points_per_chunk:
                    continue  # skip empty or too small chunks

                # Run RANSAC on chunk
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(chunk_points)

                plane_model, inliers = pcd.segment_plane(
                    distance_threshold=self.distance_threshold,
                    ransac_n=self.number_of_points_per_plane,
                    num_iterations=self.number_of_iterations,
                )

                a, b, c, d = plane_model

                # Skip nearly vertical planes
                if abs(c) < 0.5:
                    continue

                inlier_points = chunk_points[inliers]
                outlier_points = np.delete(chunk_points, inliers, axis=0)

                ground_all.append(inlier_points)
                non_ground_all.append(outlier_points)
                plane_models.append((a, b, c, d))

        # Combine all chunks
        if not ground_all:
            return points, np.empty((0, 3)), []

        return (
            np.vstack(non_ground_all),
            np.vstack(ground_all),
            plane_models,
        )
