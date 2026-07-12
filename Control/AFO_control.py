import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from matplotlib import pyplot as plt
from AFO import (Cluster,main_loop,SignalProcessor)
import numpy as np
import serial
import time
import threading
import message_filters


class walker_control_node(Node):
    def __init__(self):
        super().__init__('Adaptive_Frequency_Oscillator')

        self.current_scan = None
        self.encoder = None
        self.latest_wheel_velocity = 0.0
        self.last_sensor_update = self.get_clock().now()
        self.awaiting_confirmation = False
        self.assist_confirmed = threading.Event()
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.abs_time_history=[]

        # 3. Publishers
        self.pub_shutdown = self.create_publisher(Bool, '/shutdown', 1)
        self.pub_right_motor = self.create_publisher(Float64, '/right_wheel_velocity', 1)
        self.pub_left_motor = self.create_publisher(Float64, '/left_wheel_velocity', 1)

        # 4. Subscribers (Note the QoS profile for LiDAR), synchronized so each control tick
        # consumes a temporally-aligned scan/encoder pair instead of two independently-latest values.
        scan_sub = message_filters.Subscriber(self, LaserScan, '/scan_legs_filtered', qos_profile=qos_profile_sensor_data)
        encoder_sub = message_filters.Subscriber(self, JointState, '/encoder_data', qos_profile=1)

        self.ts = message_filters.ApproximateTimeSynchronizer([scan_sub, encoder_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.synced_callback)

        #rospy.on_shutdown(stop_motors)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Controller engaged. Please Begin Walking Calibrating Your Walking Style.")
        self.get_logger().info("=" * 50)        

        self.cluster = Cluster()
        self.main = main_loop(fs=6)
        self.signal_process=SignalProcessor()

        #self.arm_timer = self.create_timer(0.5, self.arm_hardware_callback)
        self.timer = self.create_timer(1.0 / self.main.fs, self.control_loop_callback)
        self.motor_timer = self.create_timer(1.0 / 30.0, self.motor_publish_callback)

    #Unlock ESP32 on Startup
    def arm_hardware_callback(self):
        #self.arm_timer.cancel()
        if rclpy.ok():
            arm_msg = Bool(data=False)
            self.pub_shutdown.publish(arm_msg)
            self.get_logger().info('Sent hardware unlock command to ESP32 firmware gates.')

    #Command 0 Velocity to Motors cancel callback
    def stop_motor(self):
        print("STOPPING MOTORS AND ENGAGING FIRMWARE LOCKOUT...")
        try:
            #self.arm_timer.cancel()
            self.timer.cancel()
            self.motor_timer.cancel()
            self.latest_wheel_velocity = 0.0
            stop = Float64(data=0.0)
            self.pub_left_motor.publish(stop)
            self.pub_right_motor.publish(stop)
            
            lock = Bool(data=True) 
            self.pub_shutdown.publish(lock)
        except Exception as e:
            print(f"Could not broadcast stop frame: {e}")


    def synced_callback(self, scan_msg, encoder_msg):
        self.current_scan = scan_msg
        self.encoder = encoder_msg
        self.last_sensor_update = self.get_clock().now()

    #Runs on a background thread so the ROS executor (and sensor callbacks) keep running
    #while waiting for the operator to confirm powered assist should begin.
    def _wait_for_assist_confirmation(self):
        input("Calibration complete. Press Enter to begin powered assist...")
        self.assist_confirmed.set()

    #Publishes the latest computed wheel velocity at a fixed 30Hz, decoupled from the
    #(slower) perception/control rate. Fail-safe checkpoint: zeros the command if sensor
    #data has gone stale, so a stalled control loop can't leave motors spinning indefinitely.
    def motor_publish_callback(self):
        stale = (self.get_clock().now() - self.last_sensor_update).nanoseconds / 1e9 > 0.3
        velocity = 0.0 if stale else self.latest_wheel_velocity
        self.pub_left_motor.publish(Float64(data=velocity))
        self.pub_right_motor.publish(Float64(data=velocity))


    #Control Loop
    def control_loop_callback(self):

        if self.current_scan is None:
            self.get_logger().info("no scan", throttle_duration_sec=1.0)
            return

        if self.encoder is None:
            self.get_logger().info("no encoder data", throttle_duration_sec=1.0)
            return

        try:
            self.encoder_velocity = (self.encoder.velocity[0] + self.encoder.velocity[1]) / 2.0
            self.current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
            self.abs_time_history.append(self.current_time)

            collisions = self.cluster.process_scan(self.current_scan.angle_min,self.current_scan.angle_increment,self.current_scan.ranges,0)
            self.raw_left, self.raw_right, self.isoccluded,shutdown= self.cluster.cluster_find(collisions)

            if shutdown is True:
                self.stop_motor()
                self.get_logger().error("Persistent leg occlusion. Stopping controller.")
                rclpy.shutdown()
                return

            if self.raw_left is None or self.raw_right is None:
                return

            #Hold here without advancing the AFO ramp state until the operator confirms.
            #Sensor callbacks and this loop keep running normally (unlike the old blocking
            #input()), so the eventual ramp starts from a fresh read, not a stale one.
            if self.awaiting_confirmation:
                self.latest_wheel_velocity = 0.0
                if not self.assist_confirmed.is_set():
                    return
                self.awaiting_confirmation = False
                self.pub_shutdown.publish(Bool(data=False))

            was_calibrated = self.main.calibrated

            step_result=self.main.step_from_legs(self.current_time,self.encoder_velocity,self.raw_left[0],self.raw_right[0],self.isoccluded)
        except Exception as e:
            self.get_logger().error(f"control_loop_callback failed, skipping tick: {e}", throttle_duration_sec=1.0)
            return

        if not was_calibrated and self.main.calibrated:
            self.awaiting_confirmation = True
            self.assist_confirmed.clear()
            threading.Thread(target=self._wait_for_assist_confirmation, daemon=True).start()
            self.latest_wheel_velocity = 0.0
            return

        if step_result is None or step_result[0] is None:
            return
        
        self.wheel_velocity, self.time_to_cal = step_result

        # -- Run Calculations --
        arm_msg = Bool(data=False)
        self.pub_shutdown.publish(arm_msg)
        self.latest_wheel_velocity = self.wheel_velocity


#ESP32 Hardware Reset on Shutdown
def pulse_esp32_reset():
    """Uses native serial lines with an explicit hardware settling delay to force an ESP32 reboot."""
    print("⚡ Flashing DTR/RTS lines to force firmware reboot...")
    try:
        # Open the serial hook cleanly
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        
        # 1. Drive the EN pin LOW by setting DTR/RTS states
        ser.setDTR(False)
        ser.setRTS(True)
        
        # 🔬 THE CAPACITOR DRAIN BUFFER
        # 50 milliseconds is the sweet spot: long enough to completely discharge 
        # the onboard RC filter circuit on the EN pin, but fast enough that you won't feel the lag.
        time.sleep(0.05) 
        
        # 2. Release the lines back to high to allow the chip to boot into a clean state
        ser.setDTR(True)
        ser.setRTS(False)
        
        ser.close()
        print("✅ ESP32 hardware reset successful. Motors locked.")
    except Exception as e:
        print(f"Could not signal hardware lines: {e}")


def main(args=None):
    rclpy.init(args=args)
    walker_node = walker_control_node()
    try:
        # rclpy.spin blocks here and continuously fires the timer callbacks
        rclpy.spin(walker_node)
    except KeyboardInterrupt:
        walker_node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # Stop publishing velocity commands immediately
        walker_node.stop_motor()

        #Stops running Control Loop Callback
        walker_node.timer.cancel()

        #ESP32 Connection to Motors Locked
        if hasattr(walker_node, "arm_timer"):
            walker_node.arm_timer.cancel()


        # Let the final frames exit the socket stack cleanly
        if rclpy.ok():
            rclpy.spin_once(walker_node, timeout_sec=0.2)
            
        walker_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        pulse_esp32_reset()
        # Safe shutdown procedure

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

            print(f'Calibration time: {walker_node.time_to_cal} s')
            print(f"Mean Command: {np.nanmean(walker_node.main.velocity_history):.3f} rad/s")
            print(f"Mean Encoder:      {np.nanmean(walker_node.main.encoder_data):.3f} rad/s")
            print(f"RMSE Predicted to True Velocity:       {rmse:.3f} rad/s")
            print(f'Mean error: {mean_error} --- Negative : missing low --- Positive : missing high ---')
            print(f'Mean error absolute (MAE): {mae}')
            print(f'Standard Deviation of commanded velocity {command_std}')
            print(f'Time in Active Assist: {100*(walker_node.main.control_state.count(1)/len(walker_node.main.control_state))} % ')
            print(f'Time in Active Attenuation: {100*(walker_node.main.control_state.count(2)/len(walker_node.main.control_state))} %')
            print(f'Time in Boost: {100*(walker_node.main.control_state.count(3)/len(walker_node.main.control_state))} %')
            print(f'Time in 0 Velocity: {100*(walker_node.main.control_state.count(4)/len(walker_node.main.control_state))} %')
            print(f'Time detected Frozen Gait {walker_node.main.freeze_detected_time}')

            fig,axs = plt.subplots(nrows=2, ncols=2, figsize=(10, 8))

            axs[0,0].plot(walker_node.main.commanded_timestamps, walker_node.main.velocity_history, color='red', linestyle='--', label='Velocity Command')
            axs[0,0].plot(walker_node.main.encoder_time, walker_node.main.encoder_data, color='blue', label='Encoder Data Feedback')
            axs[0,0].set_title("Velocity Command vs Encoder Data")
            axs[0,0].set_ylabel("rad/s")
            axs[0,0].set_xlabel("Time (s)")
            axs[0,0].legend()
            axs[0,0].grid(True)

            axs[0,1].plot(walker_node.main.commanded_timestamps,walker_node.main.cadence_history,color='red', linestyle='--', label='Cadence (Hz)')
            axs[0,1].set_ylabel("Hertz (Hz)")
            axs[0,1].set_xlabel("Time (s)")
            axs[0,1].set_title('Cadence History (Hz)')
            axs[0,1].legend()
            axs[0,1].grid(True)

            axs[1,0].plot(walker_node.main.encoder_time,walker_node.main.pelvis_history,color='red', linestyle='--', label='Pelvis Position (m)')
            axs[1,0].set_ylabel("meters (m)")
            axs[1,0].set_xlabel("Time (s)")
            axs[1,0].set_title("Pelvis Position (m)")
            axs[1,0].legend()
            axs[1,0].grid(True)
            
            axs[1,1].plot(walker_node.main.commanded_timestamps, walker_node.main.control_state, color='purple', linestyle='--', label='State of Control')
            axs[1,1].set_title("Control State")
            axs[1,1].set_ylabel('Control State')
            axs[1,1].set_xlabel("Time (s)")
            axs[1,1].set_yticks([1, 2, 3,4])
            axs[1,1].set_yticklabels(["Assist", "Attenuated","Boost", "Stopped"])
            axs[1,1].legend()
            axs[1,1].grid(True)
        
            plt.tight_layout()
            plt.show()

        else:
            print("\n No gait telemetry data was captured to plot.")

if __name__ == '__main__':
    main()




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