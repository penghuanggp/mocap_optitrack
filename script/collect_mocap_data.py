import argparse
import csv
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class MocapDataCollector(Node):
    def __init__(self, output_dir="data/mocap", config_path="config/mocap.yaml"):
        super().__init__("mocap_data_collector")
        self._data = {}
        self._rigid_bodies = {}
        self._sample_count = 0
        self._file_handles = {}
        self._csv_writers = {}
        self._data_dir = output_dir
        self._config_path = config_path

        # Create output directory if it doesn't exist
        os.makedirs(self._data_dir, exist_ok=True)

        # Load rigid body configuration from YAML file
        self._load_config()

        # Subscribe to rigid body topics
        for rigid_body, config in self._rigid_bodies.items():
            topic = config["pose"]
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, rb=rigid_body: self._rigid_body_callback(msg, rb),
                10,
            )
            self._data[rigid_body] = []

        self.get_logger().info(
            f"OptiTrack data collector initialized. Saving data to: {self._data_dir}"
        )

    def _load_config(self):
        """Load rigid body configuration from YAML file."""
        with open(self._config_path, "r") as f:
            config = yaml.safe_load(f)

        # Extract rigid body configurations
        self._rigid_bodies = {
            str(key): value
            for key, value in config["mocap_node"]["ros__parameters"][
                "rigid_bodies"
            ].items()
        }

    def _rigid_body_callback(self, msg, rigid_body):
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
        if rigid_body not in self._data:
            self._data[rigid_body] = []
        self._data[rigid_body].append(pose_data)
        self._sample_count += 1

    def _get_file_and_writer(self, rigid_body):
        """Get or create the file handle and CSV writer for a given rigid body."""
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
        for rigid_body in self._rigid_bodies.keys():
            poses = self._data.get(rigid_body)
            if not poses:
                self.get_logger().warning(f"No data to save for {rigid_body}")
                continue

            _, csv_writer = self._get_file_and_writer(rigid_body)

            for pose in poses:
                csv_writer.writerow(
                    [pose["timestamp"], *pose["position"], *pose["orientation"]]
                )

            self._data[rigid_body].clear()

    def destroy_node(self):
        """Override destroy_node to close file handles."""
        for rigid_body in self._file_handles.keys():
            file_handle = self._file_handles.get(rigid_body)
            if file_handle:
                file_handle.close()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description="OptiTrack Data Collector")
    parser.add_argument(
        "--output",
        default="data/mocap",
        help="Base directory to save collected data CSV files (default: data/mocap)",
    )
    parser.add_argument(
        "--config-path",
        default="config/mocap.yaml",
        help="Path to the mocap configuration file (default: config/mocap.yaml)",
    )
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    collector = MocapDataCollector(output_dir=args.output, config_path=args.config_path)

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
