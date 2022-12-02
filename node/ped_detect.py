#!/usr/bin/env python3

from __future__ import print_function

import roslib
import numpy as np
import sys
from geometry_msgs.msg import Twist
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

## Class that subscribes to image stream and can also publish velocity messages
#
# Uses OpenCV to detect crosswalk and pedestrian
class pedestrian_convertor:
   
  ## Constructor that declares which topics are being published to and subscribed to
  def __init__(self):
    self.drive_pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)
    self.cross_pub = rospy.Publisher("/crosswalk", String, queue_size=1)
    self.cross_sub = rospy.Subscriber("/crosswalk", String, self.crosswalk_callback)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.image_callback,queue_size=1,buff_size=2**24)
    # Set class member keeping track of whether cross walk is detected to 0 (no crosswalk initially)
    self.crosswalk_detected = "0"

  ## Callback function that can publish velocity commands to the drive topic
  def image_callback(self,data):
    # convert image to a format compatible with OpenCV
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
  
    # If there is not crosswalk detected, scan for red line in front of crosswalk
    if self.crosswalk_detected == "0":
      # Convert to HSV and filter out all (red) pixels w/o (0<h<10)U(170<h<180), s,v>100
      # Turn all filtered out pixels black and turn all else white. 
      cv_image = cv2.medianBlur(cv_image, 5)
      img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      ls, lv = 100, 100
      us, uv = 255, 255

      lh = 0
      uh = 10
      lower_red = np.array([lh,ls,lv])
      upper_red = np.array([uh,us,uv])
      maskLower = cv2.inRange(img_hsv, lower_red, upper_red)
      lh = 170
      uh = 180
      lower_red = np.array([lh,ls,lv])
      upper_red = np.array([uh,us,uv])
      maskUpper = cv2.inRange(img_hsv, lower_red, upper_red)
      mask = maskLower + maskUpper

      imgWidth = 1280
      imgHeight = 720
      totalRed = 0
      for i in range(imgWidth):
        if mask[imgHeight-40, i] == 255:
          totalRed += 1
      # If the red line is detected, stop the robot and publish that the crosswalk is detected
      if totalRed >= 20:
        self.cross_pub.publish("1")
        move = Twist()
        move.linear.x = 0
        move.angular.z = 0
        self.drive_pub.publish(move)

    # If the crosswalk is detected, wait until the pedestrian crosses to cross
    else:
      move = Twist()
      move.linear.x = 0
      move.angular.z = 0
      self.drive_pub.publish(move)

      # Crop away the rest of the image except for the crosswalk
      cropHeight = 50
      cropWidth = 125
      img_crop = cv_image[380:380+cropHeight,550:550+cropWidth]
      # Convert to HSV and filter out all colours except the ped's pants
      img_hsv = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
      ls, lv = 0, 0
      us, uv = 255, 255
      lh = 80
      uh = 255
      lower_vals = np.array([lh,ls,lv])
      upper_vals = np.array([uh,us,uv])
      mask = cv2.inRange(img_hsv, lower_vals, upper_vals)

      # If the pedestrian is walking across the road, drive forward for 2.5 seconds
      # and publish that there is no crosswalk detected anymore.
      totalBlue = 0
      for i in range(cropWidth):
        if mask[cropHeight//2,i] == 255:
          totalBlue += 1
      if totalBlue >= 10:
        move.linear.x = 1 / 5
        self.drive_pub.publish(move)
        rospy.sleep(2.5)
        self.cross_pub.publish("0")

  # Subscribes to crosswalk topic. If there is a new message to the topic,
  # check to see if the crosswalk is detected and set the class member to 
  # "1" if it is.
  def crosswalk_callback(self, data):
    if "1" in str(data):
      self.crosswalk_detected = "1"
      print("Crosswalk detected")
    else:
      self.crosswalk_detected = "0"
      print("Passed Crosswalk")

## Main function that keeps the simulation running until it is stopped by a user
def main(args):
  # name of node is ped_detect
  rospy.init_node('ped_detect', anonymous=True)
  pd = pedestrian_convertor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)