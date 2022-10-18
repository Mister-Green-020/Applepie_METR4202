#!/usr/bin/env python3

# Code adapted from https://github.com/UQ-METR4202/metr4202_ximea_ros/blob/main/ximea_color/src/example_camera.py

import rospy
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge, CvBridgeError
from constants import serial

# Camera serial number
SERIAL = serial

class ColourDetector() :
    def __init__(self, serial) :
        self.bridge = CvBridge()
        self.serial = serial


        self.camera_sub = rospy.Subscriber(f'/ximea_ros/ximea_{self.serial}/image_raw', Image, self.camera_callback)
        self.id_sub = rospy.Subscriber('/block_fiducials', FiducialArray, self.fiducials_callback)
        self.pub = rospy.Publisher('/block_colour', ColorRGBA, queue_size=10)


    def camera_callback(data: Image, self):
        """
        Identify colour of image?
        """
        global img
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]
        color = ColorRGBA()
        # color.a = bgr[3]
        color.r = bgr[2]
        color.g = bgr[1]
        color.b = bgr[0]
        self.pub.publish(color)


    def fiducials_callback(data: FiducialArray, self):
        """
        Get colour of specific block using FiducialArray ID
        """
        
        # self.pub.publish(color)
    

def main() :
    rospy.init_node('block_colours')
    colours = ColourDetector(SERIAL)

    rospy.Rate(10)
    rospy.spin()

if __name__ == "__main__":
    main()
    