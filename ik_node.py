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
        self.pitch_angle = 60*pi/180


    def inverse_kinematics(self, pose: Pose) : 
        rospy.loginfo(f'Got Desired Pose:\n[\n\tpos:\n{pose.position}/\nrot:\n{pose.orientation}\n]')
        
        theta_1 = atan2(pose.position.y, pose.position.x)
        
        # Determining the desired angle of joint 3 and 2.
        # Coordinates of joint 3
        joint_3_x = pose.position.x - self.l4_length*cos(self.pitch_angle)*cos(theta_1)
        joint_3_y = pose.position.y - self.l4_length*cos(self.pitch_angle)*sin(theta_1)

        joint_3_xy = sqrt(joint_3_x**2 + joint_3_y**2)
        joint_3_z = pose.position.z - self.l1_length - self.l4_length*sin(self.pitch_angle)
        
        # Determine angle of joint 3.
        cos_theta_2 = (joint_3_xy**2 + joint_3_z**2 - self.l2_length**2 - self.l3_length**2) / (2 * self.l2_length * self.l3_length)
        
        theta_3 = atan2(-sqrt(1 - cos_theta_2**2), cos_theta_2)

        # Determine angle of joint 2.
        theta_2 = atan2(joint_3_z, joint_3_xy) - atan2(self.l3_length*sin(theta_3), self.l2_length + self.l3_length*cos(theta_3))

        # Determining the desired angle of joint 4.
        theta_4 = self.pitch_angle - theta_2 - theta_3 
        
        # Convert angles to robot orientation.
        theta_2 = (pi/2) - theta_2
        theta_3 = -theta_3
        
        msg = JointState(
            header = Header(stamp=rospy.Time.now()),
            name = ['joint_1', 'joint_2', 'joint_3', 'joint_4'],
            position = [theta_1, theta_2, theta_3, theta_4]
        )

        self.pub.publish(msg)



def main(): 
    rospy.init_node('joint_publisher', anonymous=True)
    ik_node = InverseKinematics()
    rospy.spin() 

if __name__ == '__main__':
    main()       
     
