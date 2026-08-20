"""Record labeled ROS 2 ``LaserScan`` intervals for offline validation.

For each trial, press Enter when the initial position is stable. The recorder
captures a fixed interval, pauses while the participant moves, and then waits
for Enter again before capturing the final stable interval. Each CSV row
contains one complete scan with its trial and phase labels.

Example:
    python3 lidar_scan_values.py --ros-args \
        -p topic:=/scan_legs_filtered \
        -p output_path:=Validation/LiDAR/Data/bare_thigh_trial.csv

Stop the recorder with Ctrl-C. The file is flushed periodically and closed
cleanly on shutdown. Only scans received during an active interval are saved.
"""

import csv
import json
import threading
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


CSV_FIELDS = (
    "trial_index",
    "phase",
    "interval_elapsed_sec",
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
        self.declare_parameter("interval_duration_sec", 5.0)
        self.declare_parameter("flush_every_n_scans", 10)
        self.declare_parameter("max_scans", 0)

        topic = str(self.get_parameter("topic").value)
        configured_path = str(self.get_parameter("output_path").value).strip()
        flush_every = int(self.get_parameter("flush_every_n_scans").value)

        self.flush_every = max(1, flush_every)
        self.max_scans = max(0, int(self.get_parameter("max_scans").value))
        self.interval_duration_sec = max(
            0.1, float(self.get_parameter("interval_duration_sec").value)
        )
        self.scan_count = 0
        self.trial_index = 1
        self._closed = False
        self._active_phase = None
        self._interval_start_ns = None
        self._state_lock = threading.Lock()
        self._interval_finished = threading.Event()

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
        self._interval_timer = self.create_timer(0.05, self._check_interval)

        self._input_thread = threading.Thread(
            target=self._input_loop,
            name="lidar-validation-input",
            daemon=True,
        )

        self.get_logger().info(f"Recording LaserScan messages from {topic}")
        self.get_logger().info(f"Writing scans to {self.output_path.resolve()}")
        self.get_logger().info(
            f"Each stable interval lasts {self.interval_duration_sec:.1f} seconds"
        )
        self._input_thread.start()

    def _input_loop(self) -> None:
        """Guide the experimenter through initial and final stable intervals."""
        while rclpy.ok() and not self._closed:
            try:
                input(
                    f"\nTrial {self.trial_index}: place the legs at the INITIAL "
                    "position, then press Enter to record..."
                )
            except EOFError:
                self.get_logger().warning(
                    "Terminal input is unavailable; no recording interval started"
                )
                return

            if not rclpy.ok() or self._closed:
                return
            self._start_interval("initial")
            if not self._wait_for_interval():
                return

            try:
                input(
                    f"Trial {self.trial_index}: move the legs to the FINAL "
                    "position. Once stable, press Enter to record..."
                )
            except EOFError:
                return

            if not rclpy.ok() or self._closed:
                return
            self._start_interval("final")
            if not self._wait_for_interval():
                return

            self.get_logger().info(f"Trial {self.trial_index} complete")
            self.trial_index += 1

    def _wait_for_interval(self) -> bool:
        """Wait without preventing clean ROS shutdown."""
        while rclpy.ok() and not self._closed:
            if self._interval_finished.wait(timeout=0.2):
                return True
        return False

    def _start_interval(self, phase: str) -> None:
        with self._state_lock:
            self._active_phase = phase
            self._interval_start_ns = self.get_clock().now().nanoseconds
            self._interval_finished.clear()
        self.get_logger().info(
            f"Trial {self.trial_index} {phase} interval started"
        )

    def _check_interval(self) -> None:
        """Stop the active interval after its configured duration."""
        with self._state_lock:
            if self._active_phase is None or self._interval_start_ns is None:
                return
            elapsed_sec = (
                self.get_clock().now().nanoseconds - self._interval_start_ns
            ) / 1e9
            if elapsed_sec < self.interval_duration_sec:
                return
            finished_phase = self._active_phase
            self._active_phase = None
            self._interval_start_ns = None
            self._file.flush()
            self._interval_finished.set()

        self.get_logger().info(
            f"Trial {self.trial_index} {finished_phase} interval complete"
        )

    def scan_callback(self, message: LaserScan) -> None:
        """Write a scan only while a labeled interval is active."""
        receive_time_ns = self.get_clock().now().nanoseconds

        with self._state_lock:
            if self._active_phase is None or self._interval_start_ns is None:
                return
            phase = self._active_phase
            interval_elapsed_sec = (
                receive_time_ns - self._interval_start_ns
            ) / 1e9

        receive_time_sec = receive_time_ns / 1e9
        header_time_sec = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1e9
        )

        self._writer.writerow(
            {
                "trial_index": self.trial_index,
                "phase": phase,
                "interval_elapsed_sec": f"{interval_elapsed_sec:.9f}",
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
        with self._state_lock:
            if self._closed:
                return
            self._active_phase = None
            self._interval_start_ns = None
            self._interval_finished.set()
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
