#!/usr/bin/env python3

# Code adapted from https://github.com/UQ-METR4202/metr4202_ximea_ros/blob/main/ximea_color/src/example_camera.py

import rospy
import cv2
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from cv_bridge import CvBridge, CvBridgeError
from constants import serial

# Camera serial number
SERIAL = serial

class ColourDetector() :
    def __init__(self) :
        self.bridge = CvBridge()
        self.serial = SERIAL

        self.camera_sub = rospy.Subscriber(f'/ximea_ros/ximea_{self.serial}/image_raw', Image, self.camera_callback)
        self.pub = rospy.Publisher('/central_colour', ColorRGBA, queue_size=10)
        
        self.camera_height = 612
        self.camera_width = 540

    def camera_callback(self, data: Image):
        """
        Identify colour of pixel
        """

        # Given an image, we index the middle pixels
        pixel = Image[3*self.camera_height*self.camera_width/2]
        global img
        try:
            img = self.bridge.imgmsg_to_cv2(pixel, "bgr8")
        except CvBridgeError as e:
            print(e)

        bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]
        color = ColorRGBA()
        # color.a = bgr[3]
        color.r = bgr[2]
        color.g = bgr[1]
        color.b = bgr[0]

        # Use RGB values to compute a dominant colour

        self.pub.publish(color)

def main() :
    rospy.init_node('colour_detector')
    colours = ColourDetector()

    rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__' :  
    main()
    try:
        while not rospy.is_shutdown():
            if img is not None:
                cv2.imshow("Image", img)
                cv2.waitKey(1)
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    