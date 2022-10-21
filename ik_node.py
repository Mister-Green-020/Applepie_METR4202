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


def inverse_kinematics(pose: Pose) -> JointState: 
    global pub
    rospy.loginfo(f'Got Desired Pose:\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(joint_states())

def joint_states() -> JointState:
    msg = JointState(
        header = Header(stamp=rospy.Time.now()),
        name = ['joint_1','joint_2','joint_3','joint_4']
    )

    msg.position = [
        0.5,
        0.5,
        0.5,
        0.5
    ]
    return msg


def main(): 
    global pub
    pub = rospy.Publisher('joint_angles_topic', JointState, queue_size=10)
   
    sub = rospy.Subscriber('desired_pose_topic', Pose, inverse_kinematics)

    rospy.init_node('ik_node', anonymous=True)
    rospy.spin() 

if __name__ == '__main__':
    main()       
     
