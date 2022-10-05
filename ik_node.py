#! /usr/bin/env python3


import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from ast import increment_lineno
from math import atan2, pi, sqrt, cos, sin
import joint_angles

from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt

class InverseKinematics:

    def __init__(self) :
        self.pub = rospy.Publisher('desired_joint_states', JointState, queue_size=10)
        self.sub = rospy.Subscriber('new_position', Pose, self.inverse_kinematics)
        self.pitch_angle = -60*pi/180


    def inverse_kinematics(self, pose: Pose) : 
        rospy.loginfo(f'Got Desired Pose:\n[\n\tpos:\n{pose.position}/\nrot:\n{pose.orientation}\n]')
        
        robot_joint_angles = joint_angles.Joint_Angles()

        robot_joint_angles.find_joint_angles(pose.position.x, pose.position.y, pose.position.z, self.pitch_angle)

        theta_1 = robot_joint_angles.joint_1_desired_angle
        theta_2 = robot_joint_angles.joint_2_desired_angle
        theta_3 = robot_joint_angles.joint_3_desired_angle
        theta_4 = robot_joint_angles.joint_4_desired_angle
        
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
     
