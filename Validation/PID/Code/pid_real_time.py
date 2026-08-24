import serial
import time
import threading

import message_filters
import numpy as np
import rclpy
from matplotlib import pyplot as plt
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Float64
from gait_walker_interface.msg import GaitMetrics

from Control.Code.AFO_PID import Cluster, main_loop
import select
import sys

class walker_pid_node(Node):

    def __init__(self,k_p=3.0,k_i=0.0,k_d=0.0,max_linear_velocity=0.684):
        super().__init__("PID_Node")
        self.cluster = Cluster()
        self.main = main_loop()
        self.monitor_timer = self.create_timer(0.1, self.monitor)
        self.pub_right_motor = self.create_publisher(Float64, "/right_wheel_velocity", 1)
        self.pub_left_motor = self.create_publisher(Float64, "/left_wheel_velocity", 1)
        self.scan_sub = message_filters.Subscriber(self, LaserScan, "/scan_legs_filtered", qos_profile=qos_profile_sensor_data)
        self.pub_shutdown = self.create_publisher(Bool, "/shutdown", 1)
        self.encoder_sub = message_filters.Subscriber(self, JointState, "/encoder_data", qos_profile=1)



        self.latest_scan = None
        self.latest_encoder = None
        
        self.scan_sub.registerCallback(self.scan_callback)
        self.encoder_sub.registerCallback(self.encoder_callback)



        self.motors_armed = False
        self.disarm_motors()

        #Gains
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.unclipped_velocity_command=[]
        self.feedback_velocity_history=[]
        self.error_history = []
        self.results=[]
        self.integral_error = 0.0
        self.filtered_derivative = 0.0
        self.prev_error = None
        self.prev_velo = 0
        

        self.max_linear_velocity =max_linear_velocity
        self.alpha = 0.2
        self.position_deadband = 0.02
        self.prev_time = None
        
        self.trial_id = "Gains: "

    def scan_callback(self,scan_msg):
        self.latest_scan = scan_msg
        self.last_scan_recieved = self.get_clock().now()/ 1e9
        self.control_loop_callback()

    def encoder_callback(self,encoder_msg):
        self.latest_encoder = encoder_msg
        self.last_encoder_recieved = self.get_clock().now() / 1e9



    def arm_motors(self):
        # Always send zero before unlocking.
        stop = Float64(data=0.0)
        self.pub_left_motor.publish(stop)
        self.pub_right_motor.publish(stop)

        self.pub_shutdown.publish(Bool(data=False))
        self.motors_armed = True
        self.get_logger().warn("Motors armed")


    def disarm_motors(self):
        self.motors_armed = False

        stop = Float64(data=0.0)
        self.pub_left_motor.publish(stop)
        self.pub_right_motor.publish(stop)

        self.pub_shutdown.publish(Bool(data=True))
        self.get_logger().warn("Motors stopped and locked")
        
    def velocity_command(self,pelvis,dt, desired_pelvis=-0.3,wheel_radius=0.1143):
        raw_error = pelvis - desired_pelvis
        self.error_history.append(raw_error)

        #Turn off PID if in deadband
        if abs(raw_error) < self.position_deadband:
            self.error = 0.0
            self.integral_error=0
            self.filtered_derivative = 0.0

        else:
            self.error = raw_error
            if self.prev_error is not None:

                #Integral Calculation
                self.integral_error += self.error * dt

                #Derivative Calculation
                raw_derivative = (self.error - self.prev_error) / dt
                self.filtered_derivative += self.alpha * (raw_derivative - self.filtered_derivative)
            else:
                self.filtered_derivative = 0
                self.integral_error += self.error * dt
        

            
        #PID Calculation 
        feedback = self.k_p * (self.error) + self.k_i * self.integral_error + self.k_d * self.filtered_derivative
        self.prev_error = self.error


        #History
        self.unclipped_velocity_command.append((feedback )/wheel_radius)
        velocity_command = np.clip(feedback, -self.max_linear_velocity, self.max_linear_velocity)
        feedback = feedback / wheel_radius
        self.feedback_velocity_history.append(feedback)

        return velocity_command


    def monitor(self):
            """Monitor for Key Press"""
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)

            if ready :
                sys.stdin.readline() 
                print("\n==================================================")
                print("Armed")
                print("\n==================================================")
                self.arm_motors()


    def control_loop_callback(self):

        if self.latest_scan is None:
            return ("No Scan")
        if self.latest_encoder is None:
            return ("No Encoder")
        
        scan_msg = self.latest_scan
        encoder_msg = self.latest_encoder
        ranges = np.asanyarray(scan_msg.ranges, dtype=float)

        current_time = self.get_clock().now().nanoseconds/1e9

        if self.prev_time is None: 
            self.prev_time = current_time
            return
        
        dt = current_time - self.prev_time
        if dt <= 0.0 or dt > 0.5:
            self.disarm_motors()
            self.get_logger().error(f"Invalid scan interval: {dt:.3f} s")
            return
        
        self.prev_time = current_time
        
        collisions=self.cluster.process_scan(scan_msg.angle_min, scan_msg.angle_increment, ranges, 0)
        raw_left, raw_right, _, _ = self.cluster.cluster_find(collisions,eps=0.04,min_samples=4)
        if raw_left is None or raw_right is None:
            self.disarm_motors()
            return

        pelvis = (float(raw_left[0]) + float(raw_right[0]))/2
        velocity = self.velocity_command(pelvis,dt,desired_pelvis=-0.4,wheel_radius=0.1143)
        ang_velo = velocity / 0.1143

        if self.prev_velo == 0: 
            self.pub_left_motor.publish(Float64(data=0))
            self.pub_right_motor.publish(Float64(data=0))
            return

        elif abs(self.ang_velo - self.prev_velo) > 4.4:
            if ang_velo > 0:
                ang_velo = self.prev_velo + 4.4 
            if ang_velo < 0:
                ang_velo = self.prev_velo - 4.4
    
        if self.motors_armed:
            self.pub_left_motor.publish(Float64(data=ang_velo))
            self.pub_right_motor.publish(Float64(data=ang_velo))

        self.results.append((pelvis,current_time,ang_velo,encoder_msg[0],encoder_msg[3],self.trial_id,))



def main(args=None):
    rclpy.init(args=args)
    walker_node = walker_pid_node()
    try:
        # rclpy.spin blocks here and continuously fires the timer callbacks
        rclpy.spin(walker_node)
    except KeyboardInterrupt:
        walker_node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        walker_node.disarm_motors()

        if rclpy.ok():
            rclpy.spin_once(walker_node, timeout_sec=0.2)

        walker_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


        




        


        
            

