#!/usr/bin/env python3
import rospy
import pigpio
from std_msgs.msg import Bool
from constants import open_val, close_val


class Gripper() :
    def __init__(self) :
        """
        Gripper class represents the end effector actuator, which needs to
        grab a block by closing or open to allow for grabbing
        """
        self.sub = rospy.Subscriber('/desired_gripper_position', Bool, self.gripper_callback)
        self.pub = rospy.Publisher('/gripper_position', Bool, queue_size=10)
        self.rpi = pigpio.pi()
        self.rpi.set_mode(18, pigpio.OUTPUT)



    def gripper_callback(self, open: Bool):
        """
        Callback function for the gripper which will open or grip

            Parameters: 
                open: Bool set to true is gripper should open or false if 
                    it should grip
        """

        if open.data:
            self.rpi.set_servo_pulsewidth(18, open_val)
            
        else:
            self.rpi.set_servo_pulsewidth(18, close_val)
        
        self.pub.publish(open)
    
        

def main() :
    rospy.init_node('gripper')
    gripper = Gripper()

    rospy.Rate(10)
    rospy.spin()

if __name__ == "__main__":
    main()
    