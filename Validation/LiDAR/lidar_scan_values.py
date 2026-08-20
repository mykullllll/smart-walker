"""Record ROS 2 ``sensor_msgs/LaserScan`` messages for offline validation.

Each CSV row contains one complete scan. The ``ranges`` and ``intensities``
columns are JSON arrays so scan boundaries and all LaserScan metadata are
preserved.

Example:
    python3 lidar_scan_values.py --ros-args \
        -p topic:=/scan_legs_filtered \
        -p output_path:=Validation/LiDAR/Data/bare_thigh_trial.csv

Stop the recorder with Ctrl-C. The file is flushed periodically and closed
cleanly on shutdown.
"""

import csv
import json
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


CSV_FIELDS = (
    "scan_index",
    "receive_time_sec",
    "header_time_sec",
    "frame_id",
    "angle_min",
    "angle_max",
    "angle_increment",
    "time_increment",
    "scan_time",
    "range_min",
    "range_max",
    "ranges",
    "intensities",
)


class LidarScanRecorder(Node):
    """Subscribe to a LaserScan topic and write one CSV row per message."""

    def __init__(self) -> None:
        super().__init__("lidar_scan_recorder")

        self.declare_parameter("topic", "/scan_legs_filtered")
        self.declare_parameter("output_path", "")
        self.declare_parameter("flush_every_n_scans", 10)
        self.declare_parameter("max_scans", 0)

        topic = str(self.get_parameter("topic").value)
        configured_path = str(self.get_parameter("output_path").value).strip()
        flush_every = int(self.get_parameter("flush_every_n_scans").value)

        self.flush_every = max(1, flush_every)
        self.max_scans = max(0, int(self.get_parameter("max_scans").value))
        self.scan_count = 0
        self._closed = False

        if configured_path:
            self.output_path = Path(configured_path).expanduser()
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.output_path = Path(f"lidar_scans_{timestamp}.csv")

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._file.flush()

        self._subscription = self.create_subscription(
            LaserScan,
            topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"Recording LaserScan messages from {topic}")
        self.get_logger().info(f"Writing scans to {self.output_path.resolve()}")

    def scan_callback(self, message: LaserScan) -> None:
        """Write a newly received scan without resampling or duplicating it."""
        receive_time_sec = self.get_clock().now().nanoseconds / 1e9
        header_time_sec = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1e9
        )

        self._writer.writerow(
            {
                "scan_index": self.scan_count,
                "receive_time_sec": f"{receive_time_sec:.9f}",
                "header_time_sec": f"{header_time_sec:.9f}",
                "frame_id": message.header.frame_id,
                "angle_min": repr(float(message.angle_min)),
                "angle_max": repr(float(message.angle_max)),
                "angle_increment": repr(float(message.angle_increment)),
                "time_increment": repr(float(message.time_increment)),
                "scan_time": repr(float(message.scan_time)),
                "range_min": repr(float(message.range_min)),
                "range_max": repr(float(message.range_max)),
                "ranges": json.dumps(list(message.ranges), separators=(",", ":")),
                "intensities": json.dumps(
                    list(message.intensities), separators=(",", ":")
                ),
            }
        )

        self.scan_count += 1
        if self.scan_count % self.flush_every == 0:
            self._file.flush()
            self.get_logger().info(
                f"Recorded {self.scan_count} scans",
                throttle_duration_sec=2.0,
            )

        if self.max_scans and self.scan_count >= self.max_scans:
            self.get_logger().info(
                f"Reached max_scans={self.max_scans}; stopping recorder"
            )
            self.close()
            rclpy.shutdown()

    def close(self) -> None:
        """Flush and close the output file exactly once."""
        if self._closed:
            return
        self._file.flush()
        self._file.close()
        self._closed = True
        self.get_logger().info(
            f"Saved {self.scan_count} scans to {self.output_path.resolve()}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    recorder = LidarScanRecorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Recording stopped by user")
    finally:
        recorder.close()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

