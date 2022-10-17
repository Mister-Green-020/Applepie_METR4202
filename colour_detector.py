#!/usr/bin/env python3

import rospy
import pigpio
from std_msgs.msg import Bool
from fiducial_msgs.msg import FiducialArray
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge, CvBridgeError

SERIAL = '31702051'

class ColourDetector() :
    def __init__(self, serial) :
        self.bridge = CvBridge()
        self.serial = serial

        self.sub = rospy.Subscriber(f'/ximea_ros/ximea_{self.serial}/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('/block_colour', ColorRGBA, queue_size=10)
        



    def callback(data: Image, self):
        global img
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]
        color = ColorRGBA()
        color.r = bgr[2]
        color.g = bgr[1]
        color.b = bgr[0]
        self.pub.publish(color)
    
        

def main() :
    rospy.init_node('block_colours')
    colours = ColourDetector(SERIAL)

    rospy.Rate(10)
    rospy.spin()

if __name__ == "__main__":
    main()
    