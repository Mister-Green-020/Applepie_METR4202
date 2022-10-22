#!/usr/bin/env python3

# Code adapted from https://github.com/UQ-METR4202/metr4202_ximea_ros/blob/main/ximea_color/src/example_camera.py

import rospy
import cv2
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from cv_bridge import CvBridge, CvBridgeError
from constants import serial, cam_h, cam_w

# Camera serial number
SERIAL = serial

class ColourDetector() :
    def __init__(self) :
        self.bridge = CvBridge()
        self.serial = SERIAL

        self.camera_sub = rospy.Subscriber(f'/ximea_ros/ximea_{self.serial}/image_raw', Image, self.camera_callback)
        self.raw_colour_pub = rospy.Publisher('/rgba_colour', ColorRGBA, queue_size=10)
        self.pub = rospy.Publisher('/block_colour', String, queue_size=10)
        
        self.camera_height = cam_h
        self.camera_width = cam_w

    def camera_callback(self, data: Image):
        """
        Identify colour the very middle pixel where the block lays
        """

        # Given an image, we index the middle pixels
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.circle(cv_image, (int(self.camera_height/2),int(self.camera_width/2)), 30, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        bgr = cv_image[cv_image.shape[0] // 2, cv_image.shape[1] // 2, :]
        colour = ColorRGBA(
            r = bgr[2],
            g = bgr[1],
            b = bgr[0]
        )
        self.raw_colour_pub.publish(colour)

        msg = String(
            data = self.colour_identifier(colour)
        )

        self.pub.publish(msg)
    
    def colour_identifier(self, rgba : ColorRGBA) -> String :
        if (rgba.r > rgba.g and rgba.r > rgba.b and rgba.r > 150) :
            msg = 'red'
        elif (rgba.g > rgba.r and rgba.g > rgba.b and rgba.g > 150) :
            msg = 'green'
        elif (rgba.b > rgba.r and rgba.b > rgba.g and rgba.b > 150) :
            msg = 'blue'
        else :
            msg = 'yellow'
        
        return String(
            data=msg
        )

def main() :
    rospy.init_node('colour_detector')
    cd = ColourDetector()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__' :  
    main()

    