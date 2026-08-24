import time
import numpy as np
import message_filters
import rclpy
from matplotlib import pyplot as plt
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Float64
import pandas as pd
from pathlib import Path



columns = [
    "LiDAR Time Stamp (s)",
    "Encoder Time Stamp (s)",
    "Control Time (s)",
    "Scan Measurement Time (s)",
    "Encoder Measurement Time (s)"
    
]

class sensor_age(Node):

    def __init__(self):
        super().__init__("sensor_age")
        output_directory = Path(__file__).resolve().parents[1] / "control_system"
        output_directory.mkdir(parents=True, exist_ok=True)
        self.output_path = output_directory / f"sensor_age_test.csv"

        self.pub_right_motor = self.create_publisher(Float64, "/right_wheel_velocity", 1)
        self.pub_left_motor = self.create_publisher(Float64, "/left_wheel_velocity", 1)
        self.scan_sub = message_filters.Subscriber(self, LaserScan, "/scan_legs_filtered", qos_profile=qos_profile_sensor_data)
        self.pub_shutdown = self.create_publisher(Bool, "/shutdown", 1)
        self.encoder_sub = message_filters.Subscriber(self, JointState, "/encoder_data", qos_profile=1)

        self.results=[]


        self.latest_scan = None
        self.latest_encoder = None
        
        self.scan_sub.registerCallback(self.scan_callback)
        self.encoder_sub.registerCallback(self.encoder_callback)


    def export_csv(self):
        results_table = pd.DataFrame(self.results, columns=columns)
        results_table.to_csv(self.output_path, index=False)
        self.get_logger().info(f"CSV Saved to {self.output_path}")


    def scan_callback(self,scan_msg):
        self.latest_scan = scan_msg
        self.last_scan_recieved = self.get_clock().now().nanoseconds/1e9
        self.scan_measurement_time = (scan_msg.header.stamp.sec+ scan_msg.header.stamp.nanosec * 1e-9)
    
        self.control_loop_callback()

    def encoder_callback(self,encoder_msg):
        self.latest_encoder = encoder_msg
        self.last_encoder_recieved = self.get_clock().now().nanoseconds/1e9
        self.encoder_measurement_time = (encoder_msg.header.stamp.sec+ encoder_msg.header.stamp.nanosec * 1e-9)

    def control_loop_callback(self):
        if self.latest_scan is None or self.latest_encoder is None:
            return
        control_time = self.get_clock().now().nanoseconds/1e9

        self.results.append((self.last_scan_recieved,self.last_encoder_recieved,control_time,self.scan_measurement_time,self.encoder_measurement_time))
    

def main(args=None):
    rclpy.init(args=args)
    walker_node = sensor_age()
    try:
        # rclpy.spin blocks here and continuously fires the timer callbacks
        rclpy.spin(walker_node)
    except KeyboardInterrupt:
        walker_node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        walker_node.export_csv()
        walker_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

    


