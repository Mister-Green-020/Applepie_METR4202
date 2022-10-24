#!/usr/bin/env python3
# Code adapted from https://github.com/UQ-METR4202/metr4202_ximea_ros/blob/main/ximea_color/src/example_camera.py
# Code adapted from http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import string
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
        """
        Colour detector class converts the raw image of the camera to a subimage from which the RGBA colour is derived.
        """
        self.bridge = CvBridge()
        self.serial = SERIAL

        self.camera_sub = rospy.Subscriber(f'/ximea_ros/ximea_{self.serial}/image_raw', Image, self.camera_callback)
        self.raw_colour_pub = rospy.Publisher('/rgba_colour', ColorRGBA, queue_size=10)
        self.pub = rospy.Publisher('/block_colour', String, queue_size=10)
        
        self.camera_height = cam_h
        self.camera_width = cam_w

    def camera_callback(self, data: Image):
        """
        Callback function for when a new image is published, which reduces it to a circular subimage and computes colour
            Parameters:
                data: Image message type from Ximea
        """

        # Convert the Image message type to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Generate a central circle which will be where the values are taken from
        cv2.circle(cv_image, (int(self.camera_height/2),int(self.camera_width/2)), 30, 255)

        # Create a window for viewing on screen
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # Courtesy of Miguel, take the OpenCV image and extract the dominant colours of the image
        bgr = cv_image[cv_image.shape[0] // 2, cv_image.shape[1] // 2, :]
        colour = ColorRGBA(
            r = bgr[2],
            g = bgr[1],
            b = bgr[0]
        )
        
        # Publish the RGBA to a testing node to act as a debugger
        self.raw_colour_pub.publish(colour)

        msg = String(
            data = self.colour_identifier(colour)
        )

        self.pub.publish(msg)
    
    def colour_identifier(self, rgba : ColorRGBA) -> string :
        """
        Method to return one of four colours from RGBA using calibration values
            Parameters:
                rgba: ColorRGBA object containing the of rgba in range (0, 255)
            
            Returns:
                msg: Ascii string representing the four colours or 'none' if 
                     colour is not one of the four required
        """

        # Calibration values are subject to conditions
        if (rgba.r > 200 and rgba.g > 200 and rgba.b < 100) :
            msg = "'yellow'"
        elif (rgba.r > 220) :
            msg = "'red'"
        elif (rgba.b > rgba.r and rgba.g > rgba.r and rgba.g > 130 and rgba.b > 130) :
            msg = "'blue'"
        elif (rgba.g > rgba.r and rgba.g > rgba.b and rgba.g > 180) :
            msg = "'green'"
        else :
            msg = "'none'"
        
        return msg

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

    