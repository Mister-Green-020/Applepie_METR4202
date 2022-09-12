from ast import increment_lineno
from math import acos, atan2, pi, sqrt, cos, sin

from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt


class Joint_Angles:
    def __init__(self):
        # Class containing the joint angles (radians) of the robot.

        # Links lengths
        self.link_1_length = 10
        self.link_2_length = 100
        self.link_3_length = 100
        self.link_4_length = 20

        # Current angle of each joint
        self.joint_1_angle = 0
        self.joint_2_angle = 0
        self.joint_3_angle = 0
        self.joint_4_angle = 0

        # Desired angle of each joint
        self.joint_1_desired_angle = 0
        self.joint_2_desired_angle = 0
        self.joint_3_desired_angle = 0
        self.joint_4_desired_angle = 0

        # Limits for each joint
        #[INSERT LIMITS HERE]
    
    def find_joint_angles(self, x_coordinate, y_coordinate, z_coordinate, \
                        pitch_angle) -> None:
        """
        Determine the desired joint angles required to get to a desired frame using 
        inverse kinematics.
        Parameters: 
            x_coordinate (int): x coordinate of the desired frame relative to the 
            base of the arm.
            y_coordinate (int): y coordinate of the desired frame relative to the
            base of the arm.
            z_coordinate (int): z coordinate of the desired frame relative to the
            base of the arm.
            pitch_angle (int): angle of the claw in radians, at the desired frame.
        """
        # Determining the desired angle of joint 1:
        self.joint_1_desired_angle = (atan2(y_coordinate, x_coordinate)) % (2*pi)
        
        # Distance from joint 1 frame to joint 4 frame in the xy plane.
        xy_distance_to_joint_4 = sqrt(x_coordinate**2 + y_coordinate**2) - self.link_4_length
        # Distance from joint 1 frame to joint 4 frame in the z axis.
        z_distance_to_joint_4 = z_coordinate - self.link_1_length
        # Total distance from joint 1 frame to joint 4 frame.
        distance_joint_14 = sqrt(xy_distance_to_joint_4**2 + z_distance_to_joint_4**2)

        # Angle from joint 1 frame to joint 4 frame.
        theta_14 = acos(xy_distance_to_joint_4/distance_joint_14)
        
        # Angle from joint 2 frame to joint 4 frame.
        theta_24_numerator = self.link_2_length**2 + distance_joint_14**2 - self.link_3_length**2
        theta_24_denominator = 2 * self.link_2_length * distance_joint_14
        theta_24 = acos(theta_24_denominator/theta_24_denominator)

        # Determining the desired angle of joint 2:
        # Subtract from pi/4 as initial position is vertical not horizontal.
        if z_coordinate > self.link_1_length:
            # Add angles between joints 1 and 4, and 2 and 4, if the grabber is above the second 
            # joint.
            self.joint_2_desired_angle = (theta_24 + theta_14) % (2*pi)
        else:
            # Find the difference between the angle from 1 and 4, and 2 and 4 if the grabber is
            # below the second joint.
            self.joint_2_desired_angle = (theta_24 - theta_14) % (2*pi)
        
        # Determining the desired angle of joint 3.
        # Subtract from pi/2 as initial position is vertical not horizontal.
        theta_3_numerator = self.link_2_length**2 + self.link_3_length**2 - distance_joint_14**2
        theta_3_denominator = 2 * self.link_2_length * self.link_3_length
        self.joint_3_desired_angle = acos(theta_3_numerator/theta_3_denominator) % (2*pi)
        
        # Determining the desired angle of joint 4.
        if z_coordinate > self.link_1_length:
            # Subtract angles between joints 1 and 4 if the grabber is above the second joint.
            self.joint_4_desired_angle = (pi + ((pi - (theta_24 + self.joint_3_desired_angle))\
                - theta_14)) % (2*pi)
        else:
            # Add angles between joints 1 and 4 if the grabber is below the second joint.
            self.joint_4_desired_angle = (pi + ((pi - (theta_24 + self.joint_3_desired_angle))\
                + theta_14)) % (2*pi)

    
    def plot_robot(self, joint_angle_1, joint_angle_2, joint_angle_3, joint_angle_4) -> None:
        # Plots the robot configuration for debugging
        fig = plt.figure()
        axes = plt.axes(projection='3d')
        
        # Plot Link 1
        # Link 1 pivots about (0,0,0)
        link_1_x_start = 0
        link_1_y_start = 0
        link_1_z_start = 0
        link_1_x_end = 0
        link_1_y_end = 0
        link_1_z_end = self.link_1_length
        plt.plot([link_1_x_start, link_1_x_end], [link_1_y_start, link_1_y_end],\
            [link_1_z_start, link_1_z_end])

        # Plot Link 2
        # Link 2 pivots about the end point of Link 1
        link_2_x_start = link_1_x_end
        link_2_x_end = self.link_2_length * cos(joint_angle_2) * cos(joint_angle_1) + link_2_x_start
        link_2_y_start = link_1_y_end
        link_2_y_end = self.link_2_length * cos(joint_angle_2) * sin(joint_angle_1) + link_2_y_start
        link_2_z_start = link_1_z_end
        link_2_z_end = self.link_2_length * sin(joint_angle_2) + link_2_z_start
        plt.plot([link_2_x_start, link_2_x_end], [link_2_y_start, link_2_y_end],\
            [link_2_z_start, link_2_z_end])

        # Plot Link 3
        # Link 3 pivots about the end point of Link 2
        link_3_x_start = link_2_x_end
        link_3_x_end = self.link_3_length * cos(joint_angle_2+joint_angle_3) * cos(joint_angle_1) + link_3_x_start
        link_3_y_start = link_2_y_end
        link_3_y_end = self.link_3_length * cos(joint_angle_2+joint_angle_3) * sin(joint_angle_1) + link_3_y_start
        link_3_z_start = link_2_z_end
        link_3_z_end = self.link_3_length * sin(joint_angle_2+joint_angle_3) + link_3_z_start
        plt.plot([link_3_x_start, link_3_x_end], [link_3_y_start, link_3_y_end],\
            [link_3_z_start, link_3_z_end])

        # Plot Link 4
        # Link 4 pivots about the end point of Link 3
        link_4_x_start = link_3_x_end
        link_4_x_end = self.link_4_length * cos(joint_angle_2+joint_angle_3+joint_angle_4) * cos(joint_angle_1) + link_4_x_start
        link_4_y_start = link_3_y_end
        link_4_y_end = self.link_4_length * cos(joint_angle_2+joint_angle_3+joint_angle_4) * sin(joint_angle_1) + link_4_y_start
        link_4_z_start = link_3_z_end
        link_4_z_end = self.link_4_length * sin(joint_angle_2+joint_angle_3+joint_angle_4) + link_4_z_start
        plt.plot([link_4_x_start, link_4_x_end], [link_4_y_start, link_4_y_end],\
            [link_4_z_start, link_4_z_end])

        # print(link_2_x_end, link_2_y_end, link_2_z_end)
        plt.show()


def main():
    # Initilise the robot.
    robot = Joint_Angles()
    
    # print(atan2(1,2))
    # print(atan2(-1,2))
    
    # Get coordinates and pitch of the desired frame.
    x_coordinate = float(input("x_coordinate: "))
    y_coordinate = float(input("y_coordinate: "))
    z_coordinate = float(input("z_coordinate: "))
    pitch_angle = float(input("pitch_angle: "))
    
    # Find the joint angle that reach the desired frame.
    robot.find_joint_angles(x_coordinate,y_coordinate,z_coordinate,pitch_angle)

    # Print desired joint angles to the terminal
    print(robot.joint_1_desired_angle, robot.joint_2_desired_angle, robot.joint_3_desired_angle, robot.joint_4_desired_angle)
    
    # Plot the robot
    robot.plot_robot(robot.joint_1_desired_angle, robot.joint_2_desired_angle, \
        robot.joint_3_desired_angle, robot.joint_4_desired_angle)

if __name__ == '__main__':
    main()