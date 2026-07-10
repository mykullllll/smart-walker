import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from matplotlib import pyplot as plt
from AFO import (Cluster,main_loop)
import numpy as np

'''class PID:


    def pid(self,pelvis,state):
        if not state:
            self.prev_error = []
            return None
        else:
            error=self.x_d - pelvis
            self.prev_error.append(error)
            if len(self.prev_error) > 10:
                tau = 0.1
                filtered_error = tau * float(error) + (1 - tau) * float(self.prev_error[-2])
                d_term = ((filtered_error - self.prev_error[-2]) / self.dt) * self.k_d
                p_term = self.k_p * filtered_error 
                feedback_velocity = d_term  + p_term
                feedback_velocity = np.clip(feedback_velocity,0,0.5)
                return -feedback_velocity
        else:
            return None'''

        


class walker_control_node(Node):
    def __init__(self):
        super().__init__('Adaptive_Frequency_Oscillator')

        self.current_scan = None
        self.encoder = None
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # 3. Publishers
        self.pub_shutdown = self.create_publisher(Bool, '/shutdown', 1)
        self.pub_right_motor = self.create_publisher(Float64, '/right_wheel_velocity', 1)
        self.pub_left_motor = self.create_publisher(Float64, '/left_wheel_velocity', 1)
        
        # 4. Subscribers (Note the QoS profile for LiDAR)
        self.create_subscription(LaserScan, '/scan_legs_filtered', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(JointState, '/encoder_data', self.encoder_callback, 1)

        #rospy.on_shutdown(stop_motors)

        self.get_logger().info("=" * 50)
        self.get_logger().info("Controller engaged. Please Begin Walking Calibrating Your Walking Style.")
        self.get_logger().info("=" * 50)        

        self.cluster = Cluster()
        self.main = main_loop()

        self.timer = self.create_timer(1.0 / self.main.fs, self.control_loop_callback)


    def scan_callback(self, msg):
        self.current_scan = msg

    def encoder_callback(self, msg):
        self.encoder = msg

    #The Control Loop Timer (Replaces the while loop)
    def control_loop_callback(self):
        if self.current_scan is None:
            self.get_logger().info("no scan")
            return
        
        if self.encoder is None:
            self.get_logger().info("no encoder data")
            return
        
        self.encoder_velocity = (self.encoder.velocity[0] + self.encoder.velocity[1]) / 2.0
        self.current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
    
        collisions = self.cluster.process_scan(self.current_scan.angle_min,self.current_scan.angle_increment,self.current_scan.ranges,0) 
        self.raw_left, self.raw_right, self.isoccluded= self.cluster.cluster_find(collisions)
        if self.raw_left is None or self.raw_right is None:
            return
        wheel_velocity = self.main.step_from_legs(self.current_time,self.encoder_velocity,self.raw_left[0],self.raw_right[0],self.isoccluded)
        
        # -- Run Calculations -- 


                #self.pub_left_motor.publish(Float64(data=wheel_velocity))
                #self.pub_right_motor.publish(Float64(data=wheel_velocity))

def main(args=None):
    rclpy.init(args=args)
    walker_node = walker_control_node()
    try:
        # rclpy.spin blocks here and continuously fires the timer callbacks
        rclpy.spin(walker_node)
    except KeyboardInterrupt:
        walker_node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # Safe shutdown procedure
        '''stop_msg = Float64()
        stop_msg.data = 0.0
        walker_node.pub_left_motor.publish(stop_msg)
        walker_node.pub_right_motor.publish(stop_msg)'''

        if hasattr(walker_node, 'main') and len(walker_node.main.commanded_timestamps) > 0:

            velocity_history = np.array(walker_node.main.velocity_history)
            encoder_velocity = np.array(walker_node.main.encoder_data)

            start_idx=len(walker_node.main.encoder_data)-len(walker_node.main.velocity_history)
            error=velocity_history - encoder_velocity[start_idx:]
            mean_error=np.mean(error)
            mae = np.mean(np.abs(error))
            rmse = np.sqrt(np.mean(np.square(error)))
            command_std = np.std(velocity_history)

            print("\n Generating Post-Run Gait Calibration Plots...")

            print(f"Predicted mean: {np.mean(walker_node.main.velocity_history):.3f} rad/s")
            print(f"True mean:      {np.nanmean(walker_node.main.encoder_data):.3f} rad/s")
            print(f"RMSE Predicted to True Velocity:       {rmse:.3f}")
            print(f'Mean error: {mean_error} --- Negative : missing low --- Positive : missing high ---')
            print(f'Mean error absolute {mae}')
            print(f'Standard Deviation of commanded velocity {command_std}')

            plt.figure(1)
            
            # FIXED NAMESPACES: Pointing consistently to your main control object attributes
            plt.plot(walker_node.main.commanded_timestamps, walker_node.main.velocity_history, color='red', linestyle='--', label='Velocity Command')
            plt.plot(walker_node.main.encoder_time, walker_node.main.encoder_data, color='blue', label='Encoder Data Feedback')
            plt.plot(walker_node.main.commanded_timestamps, walker_node.main.control_state, color='purple', linestyle='--', label='State of Control')
            
            plt.ylabel("rad/s")
            plt.xlabel("Time (s)")
            plt.legend()
            plt.grid(True)
            plt.show() # This blocks execution until you physically close the plot window
        else:
            print("\n No gait telemetry data was captured to plot.")

        # Cleanly shut down the ROS 2 node context
        walker_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
