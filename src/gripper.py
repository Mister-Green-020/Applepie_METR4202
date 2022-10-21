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



    def gripper_callback(self, open: Bool):
        """
        Callback function for the gripper which takes in a Boolean message from a topic
        """

        if open.data:
            self.rpi.set_servo_pulsewidth(18, 2000)
            
        else:
            self.rpi.set_servo_pulsewidth(18, 1500)
        
        self.pub.publish(open)
    
        

def main() :
    rospy.init_node('gripper')
    gripper = Gripper()

    rospy.Rate(10)
    rospy.spin()

if __name__ == "__main__":
    main()
    