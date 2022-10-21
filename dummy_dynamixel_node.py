#! /usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

def callback(JointState):
    rospy.loginfo(JointState)

def subscriber():
    rospy.init_node("dynamixel_node", anonymous=True)
    rospy.Subscriber("joint_angles_topic", JointState, callback)
    rospy.spin()

try: 
    subscriber()
except rospy.ROSInterruptException: 
    pass