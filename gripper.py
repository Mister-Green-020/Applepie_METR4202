#!/usr/bin/env python3

import rospy
import pigpio
from std_msgs.msg import Bool


def gripper_callback(open: Bool):
    """
    Callback function for the gripper which takes in a Boolean message from a topic
    """
    if open:
        rpi.set_servo_pulsewidth(18, 1000)
        
    else:
        rpi.set_servo_pulsewidth(18, 1500)
    
    gripper_publisher.publish(open)
    
        

def main() :
    global rpi
    global gripper_publisher

    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT)

    rospy.init_node('gripper')
    rospy.Subscriber('/desired_gripper_position', Bool, gripper_callback)
    
    gripper_publisher = rospy.Publisher('/gripper_position', Bool, queue_size=10)
    rospy.Rate(10)
    rospy.spin()

if __name__ == "__main__":
    main()
    