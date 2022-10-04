from ast import increment_lineno
from math import atan2, pi, sqrt, cos, sin

from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt


class Joint_Angles:
    def __init__(self):
        # Class containing the joint angles (radians) of the robot.

        # Links lengths
        self.link_1_length = 70
        self.link_2_length = 115
        self.link_3_length = 95
        self.link_4_length = 70

        # Desired angle of each joint
        self.joint_1_desired_angle = 0
        self.joint_2_desired_angle = 0
        self.joint_3_desired_angle = 0
        self.joint_4_desired_angle = 0

        # Limits for each joint
        self.joint_1_limit = None
        self.joint_2_limit = 2.4
        self.joint_3_limit = 2.6
        self.joint_4_limit = 2.8
    
    def find_joint_angles(self, x_coordinate, y_coordinate, z_coordinate, \
                        pitch_angle) -> list[int, int]:
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
        # Error Handling
        # Break if desired frame is outside the workspace
        workspace_limit = self.link_2_length + self.link_3_length + \
            self.link_4_length
        distance_to_desired_frame = sqrt(x_coordinate**2 + y_coordinate**2 + \
            (z_coordinate - self.link_1_length)**2)
        if distance_to_desired_frame > workspace_limit:
            print("Error: Desired frame is out of range.")
            return None

        # Determining the desired angle of joint 1:
        self.joint_1_desired_angle = (atan2(y_coordinate, x_coordinate))\
        
        # Determining the desired angle of joint 3 and 2.
        # Coordinates of joint 3
        joint_3_x = x_coordinate - self.link_4_length*cos(pitch_angle)*\
            cos(self.joint_1_desired_angle)
        joint_3_y = y_coordinate - self.link_4_length*cos(pitch_angle)*\
            sin(self.joint_1_desired_angle)
        joint_3_xy = sqrt(joint_3_x**2 + joint_3_y**2)
        joint_3_z = z_coordinate - self.link_1_length - self.link_4_length*\
            sin(pitch_angle)
        
        # Determine angle of joint 3.
        cos_theta_2 = ((joint_3_xy**2 + joint_3_z**2 - self.link_2_length**2 -\
            self.link_3_length**2) / (2 * self.link_2_length * self.link_3_length))
        self.joint_3_desired_angle = (atan2(-sqrt(1 - cos_theta_2**2), cos_theta_2))

        # Determine angle of joint 2.
        self.joint_2_desired_angle = (atan2(joint_3_z, joint_3_xy) - \
            atan2(self.link_3_length*sin(self.joint_3_desired_angle),\
                self.link_2_length + self.link_3_length*\
                cos(self.joint_3_desired_angle)))

        # Determining the desired angle of joint 4.
        self.joint_4_desired_angle = ((pitch_angle - self.joint_2_desired_angle -\
            self.joint_3_desired_angle))  
        
        # Convert angles to robot orientation.
        self.joint_2_desired_angle = (pi/2) - self.joint_2_desired_angle
        self.joint_3_desired_angle = -self.joint_3_desired_angle
        #self.joint_4_desired_angle = (self.joint_4_desired_angle + (pi/2)) % (2*pi)
    
    def plot_robot(self, joint_angle_1, joint_angle_2, joint_angle_3, joint_angle_4) -> None:
        # Plots the robot configuration for debugging
        fig = plt.figure()
        axes = plt.axes(projection='3d')
        axes.set_ylabel("Y")
        axes.set_xlabel("X")
        axes.set_zlabel("Z")
        
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
    
    # Get coordinates and pitch of the desired frame.
    x_coordinate = float(input("x_coordinate: "))
    y_coordinate = float(input("y_coordinate: "))
    z_coordinate = float(input("z_coordinate: "))
    pitch_angle = float(input("pitch_angle: "))
    
    # Find the joint angle that reach the desired frame.
    robot.find_joint_angles(x_coordinate,y_coordinate,z_coordinate,pitch_angle)

    # Print desired joint angles to the terminal (rad)
    print(robot.joint_4_desired_angle, robot.joint_3_desired_angle, \
    robot.joint_2_desired_angle, robot.joint_1_desired_angle)
    # Print desired joint angles to the terminal (deg)
    print((robot.joint_4_desired_angle*(180/pi), robot.joint_3_desired_angle*(180/pi), \
        robot.joint_2_desired_angle*(180/pi), (robot.joint_1_desired_angle)*(180/pi)))
    
    # Plot the robot
    robot.plot_robot(robot.joint_1_desired_angle, ((pi/2)-robot.joint_2_desired_angle), \
        (-robot.joint_3_desired_angle), (robot.joint_4_desired_angle))


if __name__ == '__main__':
    main()