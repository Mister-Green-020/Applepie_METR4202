#!/usr/bin/env python3

import rospy
import pigpio
from std_msgs.msg import Bool


class Gripper() :
    def __init__(self) :
        self.sub = rospy.Subscriber('/desired_gripper_position', Bool, self.gripper_callback)
        self.pub = rospy.Publisher('/gripper_position', Bool, queue_size=10)
        self.rpi = pigpio.pi()
        self.rpi.set_mode(18, pigpio.OUTPUT)

        self.open = 1000
        self.close = 1500



    def gripper_callback(open: Bool, self):
        """
        Callback function for the gripper which takes in a Boolean message from a topic
        """

        if open:
            self.rpi.set_servo_pulsewidth(18, self.open)
            
        else:
            self.rpi.set_servo_pulsewidth(18, self.close)
        
        self.pub.publish(open)
    
        

def main() :
    rospy.init_node('gripper')
    gripper = Gripper()

    rospy.Rate(10)
    rospy.spin()

if __name__ == "__main__":
    main()
    