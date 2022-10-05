#! /usr/bin/env python3


import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from ast import increment_lineno
from math import atan2, pi, sqrt, cos, sin

from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt

class InverseKinematics:

    def __init__(self) :
        self.pub = rospy.Publisher('desired_joint_states', JointState, queue_size=10)
        self.sub = rospy.Subscriber('new_position', Pose, self.inverse_kinematics)

        self.l1_length = 70
        self.l2_length = 115
        self.l3_length = 95
        self.l4_length = 70


    def inverse_kinematics(self, pose: Pose) -> JointState: 
        rospy.loginfo(f'Got Desired Pose:\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
        
        angle_1 = (atan2(pose.y, pose.x))\
        
        # Determining the desired angle of joint 3 and 2.
        # Coordinates of joint 3
        joint_3_x = pose.x - self.l4_length*cos(pose.w)*\
            cos(angle_1)
        joint_3_y = pose.y - self.l4_length*cos(pose.w)*\
            sin(angle_1)
        joint_3_xy = sqrt(joint_3_x**2 + joint_3_y**2)
        joint_3_z = pose.z - self.l1_length - self.l4_length*\
            sin(pose.w)
        
        # Determine angle of joint 3.
        cos_theta_2 = ((joint_3_xy**2 + joint_3_z**2 - self.l2_length**2 -\
            self.l3_length**2) / (2 * self.l2_length * self.l3_length))
        angle_3 = (atan2(-sqrt(1 - cos_theta_2**2), cos_theta_2))

        # Determine angle of joint 2.
        angle_2 = (atan2(joint_3_z, joint_3_xy) - \
            atan2(self.l3_length*sin(angle_3),\
                self.l2_length + self.l3_length*\
                cos(angle_3)))

        # Determining the desired angle of joint 4.
        angle_4 = ((pose.w - angle_2 -\
            angle_3))  
        
        # Convert angles to robot orientation.
        angle_2 = (pi/2) - angle_2
        angle_3 = -angle_3
        
        msg = JointState(
            header = Header(stamp=rospy.Time.now()),
            name = ['joint_1','joint_2','joint_3','joint_4'],
            position = [angle_1, angle_2, angle_3, angle_4]
        )
        self.pub.publish(msg)



def main(): 
    rospy.init_node('joint_publisher', anonymous=True)
    InverseKinematics()
    rospy.spin() 

if __name__ == '__main__':
    main()       
     
