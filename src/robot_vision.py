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
        """
        Robot vision class represents the class responsible for locating 
        the blocks via aruco tags, determining if they are valid (can be
        reached) and if they can be picked up (conveyor is moving)
        """
        # Frame Transform from robot base to camera, needs to be filled in
        self.T_rc = np.array([
            [0, 1,  0, camera_x],
            [1, 0,  0, camera_y],
            [0, 0, -1, camera_z],
            [0, 0,  0, 1]
        ])

        self.pub = rospy.Publisher('block_positions', Pose, queue_size=10)
        self.sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.image_received)
        self.dummy_fiducial = FiducialTransform(
            fiducial_id = -1
        )
        # Require a dummy value which is always null to check when first starting
        self.previous_fiducials = [self.dummy_fiducial]



    def image_received(self, fiducialTransformArray: FiducialTransformArray) : 
        """        
        Callback function for fiducial transforms, when the tags are found.
        The position of the aruco tags are relative to the fiducial vertice 
        closest to the camera origin

        Parameter:
            fiducialTransformArray: List of all fiducials in camera view
        """
        fiducial_transforms = fiducialTransformArray.transforms
        # For collision checking (in bounds to be grabbed)
        available_poses = []

        for transform in fiducial_transforms :
            new_pose = Pose (
                position = self.camera_to_base(transform)
            )
            # Verify if the pose is valid for our specified constraints
            if self.collision_checking(new_pose) :
                available_poses.append(new_pose)
        
        # Need to verify the conveyor is not moving and there are valid block transforms
        if (not self.is_moving(fiducial_transforms) and len(available_poses) > 0) :
            # Select the first pose available
            pose = available_poses[0]
            self.pub.publish(pose)

        self.previous_fiducials = fiducial_transforms
    
    def collision_checking(self, fid_t: Pose) -> bool :
        """
        Checks if given block is unable to be grabbed based on predefined rules.
                Parameters:
                    position (Pose): Block transformation/position relative to
                        base of robot

                Returns: 
                    bool: True if no collisions, False if deemed invalid
        """
        # rospy.loginfo(fid_t.transform.translation.x*1000)
        if (fid_t.position.x > out_of_reach_x) :
            return False
        
        return True
        
    def camera_to_base(self, fid_t: FiducialTransform) -> Point:
        """
        Determine the position vector of the block relative to the robot base
        frame from the camera frame
                Parameters:
                    fid_t: FiducialTransform
                Returns:
                    p2: A point; x, y, z    
        """
        p1 = np.array([fid_t.transform.translation.x*1000, 
            fid_t.transform.translation.y*1000, fid_t.transform.translation.z*1000])
        # We are not concerned with rotation
        # R1 = np.array([[fid_t.rotation.x,0,0],[0,fid_t.rotation.y,0],[0,0,fid_t.rotation.z]])
        R1 = np.array([[1,0,0],[0,1,0],[0,0,1]])


        t_1 = mr.RpToTrans(R1, p1) # function from modern robotics
        t_2 = self.T_rc @ t_1

        R2, p2 = mr.TransToRp(t_2) # function from modern robotics

        # Apply a block offset as the position is relative to a single vertice closest to origin
        position = Point(
            x = p2[0]+block_offset,
            y = p2[1]+block_offset if p2[1]>0 else p2[1],
            # Fixing z as we changed camera zoom, this simplifies it
            z = 10
        )
        return position
    
    def is_moving(self, fid_t_arr) -> bool :
        """
        Function to determine if the conveyor is currently in motion by
        determining if fiducials have moved more than the allowed limit 
        since the last iteration

            Parameters:
                fid_t_arr: Array of FiducialTransforms currently available
            
            Returns:
                bool; false is stopped, true if moving
        """
        # Initiliase two FiducialTransforms
        fid_check = None
        prev_fid = None
        # We can only compare the same Fiducial. This O(n^2) loop will find
        # if there are any of the same blocks from the previous data set
        # available for comparison
        for fid_t in fid_t_arr :
            if (fid_check != None) :
                break
            for prev_fid_t in self.previous_fiducials :
                if fid_t.fiducial_id == prev_fid_t.fiducial_id :
                    fid_check = fid_t
                    prev_fid = prev_fid_t
                    # Break out of loop once found
                    break

        # If no like fiducials found, either the data set is incorrect
        # in which case, it should go to a more complete set OR
        # there are none available in which case the other condition
        # will apply so this doesnt matter
        if fid_check == None :
            rospy.loginfo("Not found")
            return True
        
        # Compute the cartesian distance between the previous and current position of the block
        distance = sqrt((fid_check.transform.translation.x*1000 - prev_fid.transform.translation.x*1000)**2 + \
            (fid_check.transform.translation.y*1000 - prev_fid.transform.translation.y*1000)**2)
        # Check if the distance is more than the error margin
        if distance > d_err :
            return True
        return False
        

def main(): 
    rospy.init_node('robot_vision', anonymous=True)
    rv = RobotVision()
    rospy.spin() 

if __name__ == '__main__':
    main()       
     
