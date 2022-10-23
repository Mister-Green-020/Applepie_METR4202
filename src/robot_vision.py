#! /usr/bin/env python3
from math import sqrt
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
        self.dummy_fiducial = FiducialTransform(
            fiducial_id = -1
        )
        self.previous_fiducials = [self.dummy_fiducial]



    def image_received(self, fiducialTransformArray: FiducialTransformArray) : 
        """
        http://docs.ros.org/en/kinetic/api/fiducial_msgs/html/msg/FiducialTransformArray.html
        
        Callback function for fiducial transforms (positions of block)
        """
        fiducial_transforms = fiducialTransformArray.transforms
        # For collision checking (in bounds to be grabbed)
        available_poses = []

        for transform in fiducial_transforms :
            new_pose = Pose (
                position = self.camera_to_base(transform)
            )
            if self.collision_checking(new_pose) :
                available_poses.append(new_pose)
        
        # If there are transforms available, select the one
        if (not self.is_moving(fiducial_transforms) and len(available_poses) > 0) :
            pose = available_poses[0]
            self.pub.publish(pose)

        self.previous_fiducials = fiducial_transforms
    
    def collision_checking(self, fid_t: Pose) -> bool :
        """
        Checks if given block is unable to be grabbed based on predefined rules.
                Parameters:
                    position (Transform): Block transformation/position
                    all_positions (Transform[]): Array of all blocks in view

                Returns: 
                    bool: True if no collisions, False is can collide
        """
        # rospy.loginfo(fid_t.transform.translation.x*1000)
        if (fid_t.position.x > out_of_reach_x) :
            return False
        
        return True
        
    def camera_to_base(self, fid_t: FiducialTransform) -> Point:
        """
        Determine the position vector of the block relative to the robot base frame from the camera frame
        """
        p1 = np.array([fid_t.transform.translation.x*1000, fid_t.transform.translation.y*1000, fid_t.transform.translation.z*1000])
        # R1 = np.array([[fid_t.rotation.x,0,0],[0,fid_t.rotation.y,0],[0,0,fid_t.rotation.z]])
        R1 = np.array([[1,0,0],[0,1,0],[0,0,1]])


        t_1 = mr.RpToTrans(R1, p1)
        t_2 = self.T_rc @ t_1

        R2, p2 = mr.TransToRp(t_2)
        position = Point(
            x = p2[0]+block_offset,
            y = p2[1]+block_offset if p2[1]>0 else p2[1],
            # z = p2[2]
            z = 10
        )
        return position
    
    def is_moving(self, fid_t_arr) -> bool :
        fid_check = None
        prev_fid = None
        # rospy.loginfo(fid_t_arr[0].transform.translation.x*1000)
        # rospy.loginfo(self.previous_fiducials[0].transform.translation.x*1000)
        for fid_t in fid_t_arr :
            if (fid_check != None) :
                break
            for prev_fid_t in self.previous_fiducials :
                if fid_t.fiducial_id == prev_fid_t.fiducial_id :
                    fid_check = fid_t
                    prev_fid = prev_fid_t
                    break
        # Redundancy in case where data is incorrect
        if fid_check == None :
            rospy.loginfo("Not found")
            return True
        # rospy.loginfo(fid_check.transform.translation.x*1000)
        # rospy.loginfo(prev_fid.transform.translation.x*1000)

        # if ((abs(fid_check.transform.translation.x*1000 - prev_fid.transform.translation.x*1000))>1) :
        #     rospy.loginfo("moving")
        #     return True
        # if ((abs(fid_check.transform.translation.y*1000) - abs(prev_fid.transform.translation.y*1000))>1) :
        #     return True
        distance = sqrt((fid_check.transform.translation.x*1000 - prev_fid.transform.translation.x*1000)**2 + \
            (fid_check.transform.translation.y*1000 - prev_fid.transform.translation.y*1000)**2)
        # rospy.loginfo(distance)
        if distance > d_err :
            return True
        return False
        

def main(): 
    rospy.init_node('robot_vision', anonymous=True)
    rv = RobotVision()
    rospy.spin() 

if __name__ == '__main__':
    main()       
     
