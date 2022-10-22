#!/usr/bin/env python

import rospy
import cv2
import numpy as np 
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from cv_bridge import CvBridge, CvBridgeError
from constants import *

SERIAL = serial
img = None

class CameraViewer:

  def __init__(self, serial):
    self.bridge = CvBridge()
    self.serial = serial
    self.image_sub = rospy.Subscriber("/fiducial_images", Image, self.callback)
    # self.image_sub = rospy.Subscriber(f"/ximea_ros/ximea_{self.serial}/image_raw", Image, self.callback)
    self.color_pub = rospy.Publisher("/test_color", ColorRGBA, queue_size=10)
    self.test_pub = rospy.Publisher("/cd_test", Image, queue_size=10)

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.circle(cv_image, (int(cam_h/2),int(cam_w/2)), 30, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]
    color = ColorRGBA()
    color.r = bgr[2]
    color.g = bgr[1]
    color.b = bgr[0]
    self.color_pub.publish(color)




def main():
  rospy.init_node('image_node', anonymous=True)
  viewer = CameraViewer(SERIAL)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()