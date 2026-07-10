import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState


class Motor_test(Node):
    def __init__(self):
        super().__init__('Motor_test')
        self.pub_right_motor = self.create_publisher(Float64, '/right_wheel_velocity', 1)
        self.pub_left_motor = self.create_publisher(Float64, '/left_wheel_velocity', 1)

        self.create_subscription(JointState, '/encoder_data', self.encoder_callback, 1)
        self.timer = self.create_timer(0.1, self.control_loop_callback)
        self.wheel_velocity = 2
        self.shutdown_requested = False

    def encoder_callback(self, msg):
        self.encoder = msg
    
    def stop_motor(self):
        stop=Float64(data=0.0)
        self.pub_left_motor.publish(stop)
        self.pub_right_motor.publish(stop)

    def control_loop_callback(self):

        msg = Float64(data=self.wheel_velocity)
        self.pub_left_motor.publish(msg)
        self.pub_right_motor.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = Motor_test()
    try:
        # rclpy.spin blocks here and continuously fires the timer callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.stop_motor()
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
