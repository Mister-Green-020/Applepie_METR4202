#! /usr/bin/env python3


import rospy
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image

from ast import increment_lineno
from math import atan2, pi, sqrt, cos, sin

from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt



class BlockPlanning:

    def __init__(self) :
        self.pub = rospy.Publisher('block_positions', String, queue_size=10)
        self.sub = rospy.Subscriber('ximea_ros/ximea_XXXXXXX/image_raw', Image, self.image_received)



    def image_received(self, image: Image) -> None: 
        """
        
        """
        msg = String()
        self.pub.publish(msg)
    
    def collision_cheecking() :
        """
        """



def main(): 
    rospy.init_node('block_planning', anonymous=True)
    BlockPlanning()
    rospy.spin() 

if __name__ == '__main__':
    main()       
     
