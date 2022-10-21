#!/usr/bin/env python3

import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState




def pose_pub(): 
    posePub = rospy.Publisher("desired_pose_topic", Pose, queue_size=10)
    rospy.init_node("test_node", anonymous=True) #initialise node
    #The anonymous ensures that the name is unique
    rate = rospy.Rate(10) #The rate at which ROS sleeps

    #Pose Messages:
    test_pose = Pose()
    test_pose.position.x = 10
    test_pose.position.y = 10
    test_pose.position.z = 10
    
    test_pose.orientation.x = 1
    test_pose.orientation.y = 1
    test_pose.orientation.z = 1
    test_pose.orientation.w = 1

    while not rospy.is_shutdown():
        #checks fi the program should shit down
        pose_msg = test_pose
        rospy.loginfo(pose_msg) #Logs the message string to alert when
        #it has been sent

        #Initialise the message constructor
        msg = Pose()
        msgdata = pose_msg
        posePub.publish(test_pose)
        rate.sleep()

try: 
    pose_pub()
except rospy.ROSInterruptException: 
        pass