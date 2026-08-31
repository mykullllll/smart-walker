"""Record labeled ROS 2 ``LaserScan`` intervals for offline validation.

For each trial, press Enter when the initial position is stable. The recorder
captures a fixed interval, pauses while the participant moves, and then waits
for Enter again before capturing the final stable interval. Each CSV row
contains one complete scan with its trial and phase labels.


Stop the recorder with Ctrl-C. The file is flushed periodically and closed
cleanly on shutdown. Only scans received during an active interval are saved.
"""
import json
import csv
import message_filters
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import pandas as pd
from pathlib import Path
from Control.Code.AFO_PID import Cluster
import select
from datetime import datetime
from pathlib import Path
import numpy as np


columns = [
    "Collision values",
    "Time (s)",
    "Trial ID"
    
]

class laser_scan(Node):
    def __init__(self,):
        super().__init__("lidar_scan_values")
        self.time = []
        self.scan_message = []
        self.results = []
        self.completed_trials=0
        self.trial_id = "thigh_0.1"

        self.monitor_timer = self.create_timer(0.1, self.monitor)
        output_directory = Path(__file__).resolve().parents[1] / "Control_system"
        output_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.output_path = output_directory / f"lidar_scan{timestamp}.csv"

        self.state = True
        self.current_scan = None
        self.cluster = Cluster()
        self.scan_sub = message_filters.Subscriber(self, LaserScan, "/scan_legs_filtered", qos_profile=qos_profile_sensor_data)
        self.scan_sub.registerCallback(self.control_loop_callback)

    def export_csv(self):
        results_table = pd.DataFrame(self.results, columns=columns)
        results_table.to_csv(self.output_path, index=False)
        self.get_logger().info(f"CSV Saved to {self.output_path}")


    def monitor(self):
        """Monitor for Key Press"""
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)

        if ready and self.state and self.completed_trials<2:
            sys.stdin.readline() 
            self.state = False
            print("\n==================================================")
            print("Recording data")
            print("\n==================================================")
            self.start_time = self.get_clock().now()

    def control_loop_callback(self,scan_msg):
        if self.state:
            return
        current_time = self.get_clock().now()
        elapsed_time= current_time - self.start_time
        elapsed_time = float(elapsed_time.nanoseconds / 1e9)  # Convert to seconds

        if elapsed_time >= 5.0:
            self.state = True
            self.completed_trials+=1
            print("\n==================================================")
            self.get_logger().warn(f"Recording stopped at {elapsed_time} s")
            print("\n==================================================")

            if self.completed_trials == 2:
                self.export_csv()
                self.get_logger().info("Recording session Complete")
                rclpy.shutdown()

            return

        ranges = np.asanyarray(scan_msg.ranges, dtype=float)
        if ranges.ndim != 1:
            self.get_logger().error("Scan ranges must be a 1D array.")
            return

        
        collisions=self.cluster.process_scan(scan_msg.angle_min, scan_msg.angle_increment, ranges, 0)

        self.results.append((
            json.dumps(collisions.tolist()),
            elapsed_time,
            self.trial_id,
        ))




def main(args=None):
    rclpy.init(args=args)
    laser_scan_node = laser_scan()

    try:
        rclpy.spin(laser_scan_node)

    except KeyboardInterrupt:
        laser_scan_node.get_logger().info("Keyboard Interrupt (SIGINT)")



if __name__ == "__main__":
    main()
