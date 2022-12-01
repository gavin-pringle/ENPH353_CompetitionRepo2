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

  def callback(self,data):
    # convert image to a format compatible with OpenCV
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    imgWidth = 1280
    imgHeight = 720
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    uh = 1
    us = 1
    uv = 105
    lh = 0
    ls = 0
    lv = 98
    lower_hsv1 = np.array([lh,ls,lv])
    upper_hsv1 = np.array([uh,us,uv])

    White1 = cv2.inRange(img_hsv, lower_hsv1, upper_hsv1)

    uh = 1
    us = 1
    uv = 128
    lh = 0
    ls = 0
    lv = 110
    lower_hsv2 = np.array([lh,ls,lv])
    upper_hsv2 = np.array([uh,us,uv]) 

    White2 = cv2.inRange(img_hsv, lower_hsv2, upper_hsv2)

    uh = 1
    us = 1
    uv = 210
    lh = 0
    ls = 0
    lv = 190
    lower_hsv3 = np.array([lh,ls,lv])
    upper_hsv3 = np.array([uh,us,uv])

    White3 = cv2.inRange(img_hsv, lower_hsv3, upper_hsv3)

    White = White1 + White2 + White3


    cv2.imshow("Raw", img_hsv)
    cv2.waitKey(1)

    img_blur = cv2.medianBlur(White, 5)

    cv2.imshow("BlurWhite", img_blur)
    cv2.waitKey(1)

    contours, hierarchy = cv2.findContours(img_blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contoursSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

    if(len(contoursSorted)>2 and cv2.contourArea(contoursSorted[0])>4500):
      cX = []
      for c in contoursSorted:
	      # compute the center of the contour
        M = cv2.moments(c)
        if M["m00"] > 0:
          cX.append(int(M["m10"] / M["m00"]))

      TopTwo = [] 
      TopTwo.append(contoursSorted[0])

      for i in range(1, len(cX)):
        if (abs(cX[i]-cX[0])<15):
          TopTwo.append(contoursSorted[i])

      X1 = 10000
      Y1 = 10000
      for i in range(len(TopTwo[0])):
        if TopTwo[0][i][0][0] < X1:
          X1= TopTwo[0][i][0][0]
        if TopTwo[0][i][0][1] < Y1:
          Y1= TopTwo[0][i][0][1]

      X2 = 0
      Y2 = 0
      if (len(TopTwo)>1):
        for i in range(len(TopTwo[1])):
          if TopTwo[1][i][0][0] > X2:
            X2= TopTwo[1][i][0][0]
          if TopTwo[1][i][0][1] > Y2:
            Y2= TopTwo[1][i][0][1]

      extract = cv_image[Y1:Y2+1, X1:X2+1]

      if (extract.size>0 and extract.size>0):
        cv2.imshow("Extract", extract)
        cv2.waitKey(1)

      

      contour_color = (0, 255, 0)
      contour_thick = 2
      rect = cv2.rectangle(cv_image, (X1, Y1), (X2, Y2), contour_color, contour_thick)

      cv2.imshow("Contours", rect)
      cv2.waitKey(1)

    else:
      cv2.imshow("Contours", cv_image)
      cv2.waitKey(1)



   

    

## Main function that keeps the simulation running until it is stopped by a user
def main(args):
  ic = image_converter()
  # name of node is License Plate Detection
  rospy.init_node('LicensePlateDetection', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
