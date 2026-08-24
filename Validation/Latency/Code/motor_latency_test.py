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
    "Commanded Velocity (rad/s)",
    "Encoder Callback Time (s)",
    "Command Publication Time (s)",
    "Right Encoder Value (rad/s)",
    "Left Encoder Value (rad/s)",
]

class motor_latency(Node):

    def __init__(self):
        super().__init__("motor_latency")
        output_directory = Path(__file__).resolve().parents[1] / "control_system"
        output_directory.mkdir(parents=True, exist_ok=True)
        self.output_path = output_directory / f"motor_latency.csv"

        self.pub_right_motor = self.create_publisher(Float64, "/right_wheel_velocity", 1)
        self.pub_left_motor = self.create_publisher(Float64, "/left_wheel_velocity", 1)
        self.scan_sub = message_filters.Subscriber(self, LaserScan, "/scan_legs_filtered", qos_profile=qos_profile_sensor_data)
        self.pub_shutdown = self.create_publisher(Bool, "/shutdown", 1)
        self.encoder_sub = message_filters.Subscriber(self, JointState, "/encoder_data", qos_profile=1)

        self.results=[]

        self.velocity_commanded_history = [1, 2 , 0.0]
        self.velocity = 0
        self.commanded_time = None

        self.command_index = 0

        self.latest_scan = None
        self.latest_encoder = None
        self.start_time = None
        self.command_sent = False
        self.motors_armed = False
        self.start_time = None
        
        self.scan_sub.registerCallback(self.scan_callback)
        self.encoder_sub.registerCallback(self.encoder_callback)
        self.start_time = self.get_clock().now().nanoseconds/1e9


    def arm_motors(self):
        stop = Float64(data=0.0)
        self.pub_left_motor.publish(stop)
        self.pub_right_motor.publish(stop)
        self.pub_shutdown.publish(Bool(data=False))

        self.start_time = (
            self.get_clock().now().nanoseconds / 1e9
        )
        self.motors_armed = True



    def export_csv(self):
        results_table = pd.DataFrame(self.results, columns=columns)
        results_table.to_csv(self.output_path, index=False)
        self.get_logger().info(f"CSV Saved to {self.output_path}")

    def stop_motors(self):
        self.motors_armed = False

        stop = Float64(data=0.0)
        self.pub_left_motor.publish(stop)
        self.pub_right_motor.publish(stop)
        self.pub_shutdown.publish(Bool(data=True))

        self.get_logger().warning("Motors stopped and locked")


    def scan_callback(self,scan_msg):
        self.latest_scan = scan_msg
        self.last_scan_recieved = self.get_clock().now().nanoseconds/1e9
        self.scan_measurement_time = (scan_msg.header.stamp.sec+ scan_msg.header.stamp.nanosec * 1e-9)
    
        self.control_loop_callback()

    def encoder_callback(self,encoder_msg):
        self.latest_encoder = encoder_msg
        self.last_encoder_recieved = self.get_clock().now().nanoseconds/1e9
        self.encoder_measurement_time = (encoder_msg.header.stamp.sec+ encoder_msg.header.stamp.nanosec * 1e-9)
        self.right_velo = encoder_msg.velocity[0]
        self.left_velo = encoder_msg.velocity[1]
        self.results.append((self.velocity,self.last_encoder_recieved,self.commanded_time,self.right_velo,self.left_velo))



    def control_loop_callback(self):
        if self.latest_scan is None or self.latest_encoder is None:
            return
        if not self.motors_armed:
            return
        self.velocity = self.velocity_commanded_history[self.command_index]
        self.commanded_time = self.get_clock().now().nanoseconds/1e9
        elapsed = (self.commanded_time - self.start_time)

        if self.start_time is None:
            self.start_time = self.commanded_time
            return
    
        if elapsed >= 5.0:
            if self.command_index >= len(self.velocity_commanded_history):
                self.stop_motors()
                return
            self.velocity = self.velocity_commanded_history[self.command_index]
            self.command_index += 1
            self.start_time = self.get_clock().now().nanoseconds/1e9


        self.pub_left_motor.publish(Float64(data=self.velocity))
        self.pub_right_motor.publish(Float64(data=self.velocity))


def main(args=None):
    rclpy.init(args=args)
    walker_node = motor_latency()

    try:
        confirmation = input(
            "Raise and secure the wheels. Type ARM to begin: "
        ).strip()

        if confirmation == "ARM":
            walker_node.arm_motors()
            rclpy.spin(walker_node)

    except KeyboardInterrupt:
        walker_node.get_logger().info(
            "Keyboard Interrupt (SIGINT)"
        )

    finally:
        walker_node.stop_motors()

        if rclpy.ok():
            rclpy.spin_once(walker_node, timeout_sec=0.2)

        walker_node.export_csv()
        walker_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

    


