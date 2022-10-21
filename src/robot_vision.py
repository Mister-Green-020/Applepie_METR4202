#! /usr/bin/env python3


import rospy
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform

from ast import increment_lineno
from constants import *

from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt

import modern_robotics as mr

class RobotVision:


    def __init__(self) :
        # Frame Transform from robot base to camera, needs to be filled in
        self.T_rc = np.array([
            [0, 1,  0, camera_x],
            [1, 0,  0, camera_y],
            [0, 0, -1, camera_z],
            [0, 0,  0, 1]
        ])

        self.pub = rospy.Publisher('block_positions', Pose, queue_size=10)
        
        # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
        # Will need to find a better message interpretation type
        
        # Suggestion http://wiki.ros.org/fiducials and
        # Use FiducialTransforms http://docs.ros.org/en/kinetic/api/fiducial_msgs/html/msg/FiducialTransform.html
        # Many http://docs.ros.org/en/kinetic/api/fiducial_msgs/html/msg/FiducialTransformArray.html
        self.sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.image_received)



    def image_received(self, fiducialTransformArray: FiducialTransformArray) -> None: 
        """
        http://docs.ros.org/en/kinetic/api/fiducial_msgs/html/msg/FiducialTransformArray.html
        
        Callback function for fiducial transforms (positions of block)
        """
        fiducial_transforms = fiducialTransformArray.transforms
        # For collision checking (in bounds to be grabbed)
        available_transforms = []

        for transform in fiducial_transforms :
            # Add a collision checker here
            available_transforms.append(transform)

        # If there are transforms available, select the first for now
        if (len(available_transforms) > 0) :
            selected = available_transforms[0]
            pose = Pose (
                position = self.camera_to_base(selected)
            )

            self.pub.publish(pose)

    
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
    def camera_to_base(self, fid_t: FiducialTransform) -> Point:
        '''Take camera traslation and rotation and return base rotation.postion '''
        p1 = np.array([fid_t.transform.translation.x*1000, fid_t.transform.translation.y*1000, fid_t.transform.translation.z*1000])
        # R1 = np.array([[fid_t.rotation.x,0,0],[0,fid_t.rotation.y,0],[0,0,fid_t.rotation.z]])
        R1 = np.array([[1,0,0],[0,1,0],[0,0,1]])

        t_1 = mr.RpToTrans(R1, p1)
        t_2 = self.T_rc @ t_1

        R2, p2 = mr.TransToRp(t_2)
        position = Point(
            x = p2[0]+17,
            y = p2[1]+17,
            z = p2[2]
        )
        return position

def main(): 
    rospy.init_node('robot_vision', anonymous=True)
    rv = RobotVision()
    rospy.spin() 

if __name__ == '__main__':
    main()       
     
