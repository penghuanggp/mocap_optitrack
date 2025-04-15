import argparse
import csv
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class MocapDataCollector(Node):
    def __init__(self, data_dir="data/mocap"):
        super().__init__("mocap_data_collector")
        self._data = {}
        self._rigid_bodies = set()
        self._sample_count = 0
        self._file_handles = {}
        self._csv_writers = {}
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._data_dir = os.path.join(data_dir, f"test_{timestamp}")

        # Create data directory if it doesn't exist
        os.makedirs(self._data_dir, exist_ok=True)

        # Subscribe to all rigid body topics
        self.create_subscription(
            PoseStamped, "/em1/pose", self._rigid_body_callback, 10
        )

        self.get_logger().info(
            f"OptiTrack data collector initialized. Saving data to: {self._data_dir}"
        )

    def _rigid_body_callback(self, msg):
        """Callback for processing rigid body pose messages"""
        # Store pose data
        pose_data = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "position": [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            "orientation": [
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ],
        }
        if "em1" not in self._data:
            self._data["em1"] = []
        self._data["em1"].append(pose_data)
        self._sample_count += 1

    def _get_file_and_writer(self):
        """Get or create the file handle and CSV writer for em1."""
        rigid_body = "em1"
        if rigid_body not in self._file_handles:
            filename = os.path.join(self._data_dir, f"{rigid_body}.csv")
            file_exists = os.path.exists(filename)

            self._file_handles[rigid_body] = open(filename, "a", newline="")
            self._csv_writers[rigid_body] = csv.writer(self._file_handles[rigid_body])

            if not file_exists:
                self._csv_writers[rigid_body].writerow(
                    [
                        "timestamp",
                        "pos_x",
                        "pos_y",
                        "pos_z",
                        "quat_w",
                        "quat_x",
                        "quat_y",
                        "quat_z",
                    ]
                )
        return self._file_handles[rigid_body], self._csv_writers[rigid_body]

    def _save_to_csv(self):
        """Save collected data to CSV files, appending to existing files."""
        rigid_body = "em1"
        poses = self._data.get(rigid_body)
        if not poses:
            self.get_logger().warning(f"No data to save for {rigid_body}")
            return

        file_handle, csv_writer = self._get_file_and_writer()

        for pose in poses:
            csv_writer.writerow(
                [pose["timestamp"], *pose["position"], *pose["orientation"]]
            )

        self._data[rigid_body].clear()

    def destroy_node(self):
        """Override destroy_node to close file handles."""
        rigid_body = "em1"
        file_handle = self._file_handles.get(rigid_body)
        if file_handle:
            file_handle.close()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description="OptiTrack Data Collector")
    parser.add_argument(
        "--data-dir",
        default="data/ground_truth",
        help="Base directory to save collected data (default: data/ground_truth)",
    )
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    collector = MocapDataCollector(data_dir=args.data_dir)

    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info("Shutting down...")
        collector._save_to_csv()
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
