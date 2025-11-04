import numpy as np
class Filter:
    def __init__(
        self,
        vehicle_min_x=0.0,
        vehicle_max_x=2.9,
        vehicle_min_y=-1.0,
        vehicle_max_y=1.0,
        cone_height=0.45,
        voxel_size=[0.1, 0.1, 0.1]
    ):
        self.vehicle_min_x = vehicle_min_x
        self.vehicle_max_x = vehicle_max_x
        self.vehicle_min_y = vehicle_min_y
        self.vehicle_max_y = vehicle_max_y
        self.cone_height = cone_height
        self.voxel_size = voxel_size

    def vehicle_filter(self, points):
        filtered_mask = (points[:, 0] < self.vehicle_min_x) | \
                        (points[:, 0] > self.vehicle_max_x) | \
                        (points[:, 1] < self.vehicle_min_y) | \
                        (points[:, 1] > self.vehicle_max_y)
        return points[filtered_mask]
    
    def air_filter(self, points):
        filtered_mask = points[:, 2] < self.cone_height
        return points[filtered_mask]
    
    def voxel_filter(self, points):
        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)
        dtype = np.dtype((np.void, voxel_indices.dtype.itemsize * voxel_indices.shape[1]))
        voxel_keys = voxel_indices.view(dtype)
        _, unique_indices, inverse_indices = np.unique(voxel_keys, return_index=True, return_inverse=True)
        filtered_points = np.zeros((len(unique_indices), 3))
        for i in range(3):
            filtered_points[:, i] = np.bincount(inverse_indices, weights=points[:, i])[unique_indices] / \
                                    np.bincount(inverse_indices)[unique_indices]
        return filtered_points
