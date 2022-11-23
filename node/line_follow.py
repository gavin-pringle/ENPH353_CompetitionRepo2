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
#  Uses OpenCV to detect line 
class image_converter:

  ## Constructor that declares which topics are being published to and subscribed to
  def __init__(self):
    self.drive_pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback, queue_size=3)

  ## Callback function that can publish velocity commands to the drive topic
  def callback(self,data):
    # convert image to a format compatible with OpenCV
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Convert to HSV and filter out all pixels w/o 0<s<5 & 100<v<255. Accept all hues.
    # Turn all filtered out pixels black and turn all else white. 
    cv_image = cv2.medianBlur(cv_image, 5)
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    uh = 255
    us = 5
    uv = 255
    lh = 0
    ls = 0
    lv = 100
    lower_hsv = np.array([lh,ls,lv])
    upper_hsv = np.array([uh,us,uv])
    mask = cv2.inRange(img_hsv, lower_hsv, upper_hsv)

    imgWidth = 1280
    imgHeight = 720
    lineHeight = round(imgHeight * 0.8)

    # Find the coordinate of the average of white pixels in the vicinity of
    # lineHeight on the LEFT side of the image.
    coordLarr = [0] * 10
    leftSum = 0
    for k in range(10):
      lineSum = 0
      numWhite = 0
      for i in range(0, imgWidth//2):
        pixel = mask[lineHeight + k, i]
        if pixel == 255:
          lineSum += i
          numWhite += 1
      if numWhite == 0:
        coordLarr[k] = 0
      else: 
        coordLarr[k] = lineSum // numWhite
      leftSum += numWhite
    # Find the average x coordinate
    coordL = sum(coordLarr) // len(coordLarr)

    # Find the coordinate of the average of white pixels in the vicinity of
    # lineHeight on the RIGHT side of the image.
    coordRarr = [0] * 10
    rightSum = 0
    for k in range(10):
      lineSum = 0
      numWhite = 0
      for i in range(imgWidth//2,imgWidth):
        pixel = mask[lineHeight + k, i]
        if pixel == 255:
          lineSum += i
          numWhite += 1
      if numWhite == 0:
        coordLarr[k] = imgWidth//2
      else:
        coordRarr[k] = lineSum // numWhite
      rightSum += numWhite
    # Find the average x coordinate
    coordR = sum(coordRarr)//len(coordRarr)

    if leftSum <= 300:
      if rightSum <= 300 or (coordL <= 100 and coordR >= imgWidth - 100):
        # If cannot detect either line, line is not detected
        coord = -1
      elif rightSum >= 700:
        # If left line weak and right line strong, hard turn left
        coord = 0
      else:
        # If can detect right line but not left line, follow right line
        coord = (coordR-(imgWidth//2)) * 2
    elif rightSum <= 300:
      if leftSum >= 700:
        # If right line weak and left line strong, hard turn right
        coord = imgWidth
      else:
        # If can detect left line but not right line, follow left line
        coord = coordL * 2
    elif leftSum >= 700:
      # If can detect both lines but left line too strong, follow right line
      coord = (coordR-(imgWidth//2)) * 2
    elif rightSum >= 700:
      # If can detect both lines but right line too strong, follow left line
      coord = coordL * 2
    else:
      # If can detect both lines well, average them
      coord = coordL + coordR-(imgWidth//2)

    # Array to show where robot thinks the road is
    state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    for i in range(10):
      if coord >= i * 128 and coord <= (i + 1) * 128:
        state[i] = 1
        break
    
    # Show Processed image of what robot sees and where it thinks the road is
    cv2.putText(mask, text=str(leftSum) + "  " + str(rightSum) + "  " + str(state), org=(20, 20), 
                fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, color=(255, 100, 255),thickness=1)
    cv2.imshow("raw", mask)
    cv2.waitKey(3)
    
    move = Twist()
    if coord == -1:
      # If the road is not detected, curve left 
      move.angular.z = 1
      print("road not detected")
    else:
      # If a coordinate is known, find the error and turn toward the coordinate
      error = coord - (imgWidth//2)
      move.angular.z = -1 * error // 100
      print("error: "+str(error))

    # Move forward at a constant speed of 1/5
    move.linear.x = 1 / 5
    
    # publish drive commands
    try:
      self.drive_pub.publish(move)
    except CvBridgeError as e:
      print(e)

## Main function that keeps the simulation running until it is stopped by a user
def main(args):
  ic = image_converter()
  # name of node is line_detector
  rospy.init_node('line_detector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
