import rclpy
import os.path
import sqlite3
from sqlite3.dbapi2 import Connection

from rclpy.node import Node
import rclpy.serialization as serialization

from labeling_msgs.srv import FrameRequest, RosbagInfoRequest
from sensor_msgs.msg import PointCloud2


# This class loads .db3 rosbag files from the file system and deserializes the data assuming it is PointCloud2
class DB3FrameLoaderNode(Node):
    def __init__(self):
        super().__init__("rosbag_frame_loader")
        self._frame_request_service = self.create_service(FrameRequest, "frame_request", self.frame_request_callback)
        self._rosbag_info_request_service = self.create_service(
            RosbagInfoRequest, "rosbag_info_request", self.rosbag_info_request_callback
        )

    # Once a FrameRequest service request is made load the corresponding .db3 file and extract the PointCloud2 data at the frame index
    def frame_request_callback(
        self, request: FrameRequest.Request, response: FrameRequest.Response
    ) -> FrameRequest.Response:
        db3_file_name = request.file.data
        frame_id = request.frame

        # Load the DB3 file, if it does not exist return invalid status code
        connection = self.load_db3_file(db3_file_name)
        if connection is None:
            response.status = False
            return response

        # Load the PointCloud2 message at the given frame id
        pointcloud = self.get_pointcloud_frame(connection, frame_id)
        connection.close()

        # If there is no valid pointcloud, return invalid status code
        if pointcloud is None:
            response.status = False
            return response

        # Everything loaded correctly
        response.status = True
        response.pointcloud = pointcloud
        return response

    # Once a FrameRequest service request is made load the corresponding .db3 file and extract the PointCloud2 data at the frame index
    def rosbag_info_request_callback(
        self, request: RosbagInfoRequest.Request, response: RosbagInfoRequest.Response
    ) -> RosbagInfoRequest.Response:
        db3_file_name = request.file.data

        # Load the DB3 file, if it does not exist return invalid status code
        connection = self.load_db3_file(db3_file_name)
        if connection is None:
            response.status = False
            return response

        # Fetch the number of frames in the rosbag
        number_of_frames = self.get_number_of_frames(connection)
        connection.close()

        # If there is no valid number of frames, return invalid status code
        if number_of_frames is None:
            response.status = False
            return response

        # Everything loaded correctly
        response.status = True
        response.number_of_frames = number_of_frames
        return response

    # Setup of a sqlite3 connection with the db3 file if it exists otherwise return None
    def load_db3_file(self, file: str) -> Connection:
        if os.path.isfile(file) and file.endswith(".db3"):
            return sqlite3.connect(file)
        return None

    # Check if the database's `messages` table exists
    def is_valid_connection(self, connection: Connection) -> bool:
        if connection is not None:
            return (
                len(
                    connection.cursor()
                    .execute("SELECT name FROM sqlite_master WHERE type='table' AND name='messages';")
                    .fetchall()
                )
                > 0
            )
        return False

    # Return the number of rows from the `messages` table
    def get_number_of_frames(self, connection: Connection) -> int:
        if self.is_valid_connection(connection):
            number_of_frames = connection.cursor().execute("SELECT COUNT(id) FROM messages").fetchone()[0]
            return number_of_frames
        return None

    # Fetch the PointCloud stored as a BLOB and deserialize it using ROS2 serialization
    # This assumes the BLOB is of type PointCloud to deserialize properly
    def get_pointcloud_frame(self, connection: Connection, frame_id: int) -> PointCloud2:
        if self.is_valid_connection(connection):
            rows = connection.cursor().execute("SELECT * FROM messages WHERE id=? LIMIT 1", (str(frame_id),))
            row = rows.fetchone()
            raw = row[3]
            return serialization.deserialize_message(raw, PointCloud2)
        return None


def main():
    rclpy.init()

    db3_frame_loader_node = DB3FrameLoaderNode()
    rclpy.spin(db3_frame_loader_node)

    db3_frame_loader_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
