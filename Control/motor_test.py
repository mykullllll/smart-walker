from std_msgs.msg import Float64, Bool
import rospy


if __name__== "__main__":
    velocity_command = 2.0

    fs= 10
    rate=rospy.Rate(fs)

    pub_shutdown = rospy.Publisher('/shutdown',Bool,queue_size=1)
    pub_right_motor = rospy.Publisher('/right_wheel_velocity', Float64, queue_size=1)
    pub_left_motor = rospy.Publisher('/left_wheel_velocity', Float64, queue_size=1)
    pub_shutdown.publish(False)

    while not rospy.is_shutdown():
        pub_left_motor.publish(Float64(velocity_command))
        pub_right_motor.publish(Float64(velocity_command))
        rate.sleep()


    pub_right_motor.publish(Float64(0))
    pub_left_motor.publish(Float64(0))
    pub_shutdown.publish(True)

    


