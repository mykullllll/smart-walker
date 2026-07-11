import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState

class Motor_test(Node):
    def __init__(self):
        super().__init__('Motor_test')
        # Setup Publishers & Subscribers
        self.pub_right_motor = self.create_publisher(Float64, '/right_wheel_velocity', 1)
        self.pub_left_motor = self.create_publisher(Float64, '/left_wheel_velocity', 1)
        self.pub_shutdown = self.create_publisher(Bool, '/shutdown', 1)
        self.create_subscription(JointState, '/encoder_data', self.encoder_callback, 1)

        self.wheel_velocity = 2.0
        self.shutdown_requested = False

        # ONE-SHOT TIMER: This calls arm_hardware_callback after 0.5 seconds without freezing the node
        self.arm_timer = self.create_timer(0.5, self.arm_hardware_callback)
        
        # Main continuous control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop_callback)

    def arm_hardware_callback(self):
        # Cancel the timer immediately so it only runs ONCE at startup
        self.arm_timer.cancel()
        
        if rclpy.ok():
            arm_msg = Bool(data=False)
            self.pub_shutdown.publish(arm_msg)
            self.get_logger().info('Sent hardware unlock command to ESP32 firmware gates.')

    def encoder_callback(self, msg):
        self.encoder = msg
    
    def stop_motor(self):
        print("STOPPING MOTORS AND ENGAGING FIRMWARE LOCKOUT...")
        try:
            stop = Float64(data=0.0)
            self.pub_left_motor.publish(stop)
            self.pub_right_motor.publish(stop)
            
            lock = Bool(data=True)
            self.pub_shutdown.publish(lock)
        except Exception as e:
            print(f"Could not broadcast stop frame: {e}")

    def control_loop_callback(self):
        if rclpy.ok():
            # Heartbeat arming message keeping the firmware loop unlocked
            arm_msg = Bool(data=False)
            self.pub_shutdown.publish(arm_msg)

            # Send velocity updates
            msg = Float64(data=self.wheel_velocity)
            self.pub_left_motor.publish(msg)
            self.pub_right_motor.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = Motor_test()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # Stop publishing velocity commands immediately
        node.timer.cancel()
        
        # Deploy safety brakes
        node.stop_motor()
        
        # Let the final frames exit the socket stack cleanly
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()