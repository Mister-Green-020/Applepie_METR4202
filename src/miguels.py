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
    global img

    # rospy.loginfo(len(data.data))
    data.data = data.data[int(3 * cam_h * cam_w / 2) : int(3 * cam_h * cam_w/2 + (3*20))]
    data.height = 20
    data.width = 1

    data.step = 3
    print(len(data.data))
    # rospy.loginfo(f"{data.height}")
    # rospy.loginfo(f"{data.width}")
    self.test_pub.publish(data)
    
    try:
      img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    


    bgr = img[img.shape[0] // 2, img.shape[1] // 2, :]
    color = ColorRGBA()
    color.r = bgr[2]
    color.g = bgr[1]
    color.b = bgr[0]
    viewer.color_pub.publish(color)
    # msg = String(
    #   data=f'{len(data.data)}'
    # )
    # self.test_pub.publish(msg)



if __name__ == '__main__':  
  rospy.init_node('image_node', anonymous=True)
  viewer = CameraViewer(SERIAL)
  try:
    while not rospy.is_shutdown():
        if img is not None:
            cv2.imshow("Image", img)
            cv2.waitKey(1)
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()