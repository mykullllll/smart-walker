from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import pandas as pd
import rclpy
import os 
output_directory = os.path.expanduser('~/ros2_ws/validation/ramp_rate')
os.makedirs(output_directory,exist_ok=True)

class ramprate(Node):

    def __init__(self):
        super().__init__('ramprate')
        
        self.encoder_sub = self.create_subscription(JointState,'encoder_data',self.encoder_callback,10)
        self.right_wheel_pub = self.create_publisher(Float64,'/right_wheel_velocity',10)
        self.left_wheel_pub = self.create_publisher(Float64,"/left_wheel_velocity",10)

        self.shutdown_pub = self.create_publisher(Bool, "/shutdown", 1)
        self.shutdown_pub.publish(Bool(data=False))

    
        self.command_index = 0
        self.motor_values = [0.0,2.0,4.0,6.0,3.0,1.0,-1.0,-3.0,-5.0,-1.0,3.0,7.0]
        self.timer = self.create_timer(0.1,self.control_loop)


        self.start_time = self.get_clock().now()
        self.motor_change_time = self.start_time.nanoseconds/1e9
        self.current_time = 0.0
        self.commanded_history = []
        self.commanded_time_history = []
        self.encoder_latency = []
        self.encoder_time_history=[]
        self.right_encoder_history = []
        self.left_encoder_history = []
        self.right_acceleration = []
        self.left_acceleration = []
        self.right_jerk = []
        self.left_jerk = []
 

    def encoder_callback(self,msg):
        self.right_encoder_velocity = msg.velocity[0]
        self.left_encoder_velocity = msg.velocity[1]
        self.current_time_encoder = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        self.recieve_time = self.get_clock().now().nanoseconds / 1e9
        self.encoder_time_history.append(self.current_time_encoder)
        self.capture_time = (msg.header.stamp.sec +msg.header.stamp.nanosec * 1e-9)     #Exact time stamp of sensor scan
        self.right_encoder_history.append(self.right_encoder_velocity)
        self.left_encoder_history.append(self.left_encoder_velocity)
        self.encoder_latency.append(self.recieve_time - self.capture_time)

    def motor_timer_callback(self):
        self.motor_change_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def control_loop(self): 
        # 5 Seconds per motor command
        self.current_time = (self.get_clock().now()- self.start_time).nanoseconds / 1e9
        if  self.current_time - self.motor_change_time > 5:
            self.command_index += 1
            self.motor_timer_callback()

        if self.command_index >= len(self.motor_values):
            self.right_wheel_pub.publish(Float64(data=0.0))
            self.left_wheel_pub.publish(Float64(data=0.0))
            raise SystemExit
        


        right_msg = Float64()
        left_msg = Float64()
        right_msg.data = float(self.motor_values[self.command_index])
        left_msg.data = float(self.motor_values[self.command_index])
        self.commanded_history.append(self.motor_values[self.command_index])
        self.commanded_time_history.append(self.current_time)
        self.right_wheel_pub.publish(right_msg)
        self.left_wheel_pub.publish(left_msg)
        
        
    def export_csv(self):
        for index in range(len(self.right_encoder_history)-1):
            self.right_acceleration.append((self.right_encoder_history[index+1]-self.right_encoder_history[index])/(self.encoder_time_history[index+1]-self.encoder_time_history[index]))
            self.left_acceleration.append((self.left_encoder_history[index+1]-self.left_encoder_history[index])/(self.encoder_time_history[index+1]-self.encoder_time_history[index]))
        for index in range(len(self.right_acceleration)-1):
            self.left_jerk.append((self.left_acceleration[index+1]-self.left_acceleration[index])/(self.encoder_time_history[index+1]-self.encoder_time_history[index]))
            self.right_jerk.append((self.right_acceleration[index+1]-self.right_acceleration[index])/(self.encoder_time_history[index+1]-self.encoder_time_history[index]))


        df=pd.DataFrame({"Commanded Velocity":self.commanded_history,
                         "Commanded Time History":self.commanded_time_history,
                         })
        
        df.to_csv(f'{output_directory}/truth_0.1.csv',index=False)
        jerk = pd.DataFrame({"Encoder Time":self.encoder_time_history,
                                "Right Encoder Velocity History":self.right_encoder_history,
                                "Left Encoder Velocity History":self.left_encoder_history,
                                "Latency Encoder":self.encoder_latency,

        })
        dt=jerk["Encoder Time"].diff()
        jerk["Right Encoder Acceleration"] = jerk["Right Encoder Velocity History"].diff()/dt
        jerk["Left Encoder Acceleration"] = jerk["Left Encoder Velocity History"].diff()/dt
        jerk["Right Encoder Jerk"] = jerk["Right Encoder Acceleration"].diff()/dt
        jerk["Left Encoder Jerk"] = jerk["Left Encoder Acceleration"].diff()/dt
        jerk.to_csv(f"{output_directory}/jerk_accel_0.1.csv", index = False)


def main(args=None):
    rclpy.init(args=args)
    node = ramprate()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.export_csv()
        node.destroy_node()
        rclpy.shutdown()

if __name__== '__main__':
    main()




        
        
        



