import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


# Custom parsing function to speed up loading the raw point data in a (width*height, 5) numpy array
def parse_pointcloud_msg(msg: PointCloud2) -> np.ndarray:
    # The stride when parsing the values. 4 bytes per float32 and 2 bytes for unit16
    stride = msg.point_step

    # Hard coding the first 12 bytes, if we have less bytes than this the pointcloud is not valid
    # Read the x, y, z and intensity values as a float32 (np.dtype(`f`)), with the correct 4 byte offset
    xs = np.ndarray((msg.width * msg.height,), np.dtype("f"), msg.data, 0, (stride,))
    ys = np.ndarray((msg.width * msg.height,), np.dtype("f"), msg.data, 4, (stride,))
    zs = np.ndarray((msg.width * msg.height,), np.dtype("f"), msg.data, 8, (stride,))

    # Read intensities only if there are enough values for this
    intensities = None
    if msg.point_step >= 16:
        intensities = np.ndarray((msg.width * msg.height,), np.dtype("f"), msg.data, 12, (stride,))
    else:
        intensities = np.zeros((msg.width * msg.height,), np.dtype("f"))

    # Read the ring value as a unit16 (np.dtype(`H`)), from the remaining 2 bytes (if there are remaining 2 points)
    rings = None
    if msg.point_step >= 18:
        rings = np.ndarray((msg.width * msg.height,), np.dtype("H"), msg.data, 16, (stride,))
    else:
        rings = np.zeros((msg.width * msg.height,), np.dtype("H"))
    # Find the nan indices to drop
    nan_indices = ~np.isnan(xs)

    # Stack the arrays column wise so we end up with rows of 5 values
    return np.stack(
        (xs[nan_indices], ys[nan_indices], zs[nan_indices], intensities[nan_indices], rings[nan_indices]), axis=-1
    )


def to_pointcloud_msg(points: np.ndarray, frame_id="map", timestamp=None) -> PointCloud2:
    # All fields in the data are of type float-32
    N = len(points)
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    # Declare four fields in the pointcloud data: x, y, z and point intensity
    fields = fields = [
        PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate(["x", "y", "z", "intensity"])
    ]

    # Construct the PointCloud2 message
    return PointCloud2(
        header=Header(frame_id=frame_id, stamp=timestamp),
        height=1,
        width=N,
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=itemsize * len(fields),
        row_step=itemsize * len(fields) * N,
        data=np.array(points, dtype=dtype).tobytes(),
    )
