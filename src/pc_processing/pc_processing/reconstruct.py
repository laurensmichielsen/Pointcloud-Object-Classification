import numpy as np

class ReconstructCone:

    def __init__(
        self,
        width_threshold: float = 0.1475,
        max_height_threshold: float = 0.45,
        min_height_threshold: float = 0.05,
        safety_factor: float = 1.15,
        include_base: bool = False,
        base_buffer: float = 0.05
    ):
        self.width_threshold = width_threshold
        self.max_height_threshold = max_height_threshold
        self.safety_factor = safety_factor
        self.include_base = include_base
        self.base_buffer = base_buffer
        self.min_height_threshold = min_height_threshold
        if include_base:
            self.min_height_threshold -= base_buffer

    def reconstruct(
        self,
        points: np.ndarray,  # Nx4 with intensity as last column
        centroids: np.ndarray
    ):
        points_in_boxes = []
        print(f"Shape of the points: {points.shape}")
        w_s = self.width_threshold * self.safety_factor
        l_s = w_s  # square base

        for i, (cx, cy, cz) in enumerate(centroids):
            x_min = cx - w_s
            x_max = cx + w_s
            y_min = cy - l_s
            y_max = cy + l_s

            # Correct parentheses for bitwise AND
            mask_cluster = (
                (points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
                (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
            )


            points_full_cluster = points[mask_cluster]
            points_in_boxes.append(points_full_cluster)

        return points_in_boxes
