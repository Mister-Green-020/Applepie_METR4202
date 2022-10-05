#! /usr/bin/env python3


import rospy
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransformArray

from ast import increment_lineno
from math import atan2, pi, sqrt, cos, sin

from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt

class BlockPlanning:


    def __init__(self, serial) :
        self.serial = serial
        self.pub = rospy.Publisher('block_positions', Pose, queue_size=10)
        
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
        # Will need to find a better message interpretation type
        
        # Suggestion http://wiki.ros.org/fiducials and
        # Use FiducialTransforms http://docs.ros.org/en/kinetic/api/fiducial_msgs/html/msg/FiducialTransform.html
        # Many http://docs.ros.org/en/kinetic/api/fiducial_msgs/html/msg/FiducialTransformArray.html
        self.sub = rospy.Subscriber(f'ximea_ros/ximea_{self.serial}/image_raw', Image, self.image_received)



    def image_received(self, fiducialTransformArray: FiducialTransformArray) -> None: 
        """
        http://docs.ros.org/en/kinetic/api/fiducial_msgs/html/msg/FiducialTransformArray.html
        """
        fiducial_transforms = fiducialTransformArray.transforms
        available_transforms = []

        for transform in fiducial_transforms :
            if self.collision_checking(transform, fiducial_transforms) :
                available_transforms.append(transform)

        if not len(available_transforms) :
            selected = available_transforms[0]
            msg = Pose()
            self.pub.publish(msg)

    
    def collision_checking(self, position, all_positions) -> bool :
        """
        Checks if given block is unable to be grabbed based on predefined rules.
                Parameters:
                    position (Transform): Block transformation/position
                    all_positions (Transform[]): Array of all blocks in view

                Returns: 
                    bool: True if no collisions, False is can collide
        """
        if (1) :
            return True
        
        return False



def main(): 
    rospy.init_node('block_planning', anonymous=True)
    bp = BlockPlanning('31702951')
    rospy.spin() 

if __name__ == '__main__':
    main()       
     
