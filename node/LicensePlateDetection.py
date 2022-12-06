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


    # cv2.imshow("Raw", img_hsv)
    # cv2.waitKey(1)

    img_blur = cv2.medianBlur(White, 5)

    # cv2.imshow("BlurWhite", img_blur)
    # cv2.waitKey(1)

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


      #Perspective Transform Points!
      #X1,Y1 top left point
      #X2,Y2 bottom right point
      #X3,Y3 top right point
      #X4,Y4 bottom left point


      X1 = 10000

      if (len(TopTwo)>1):
        for i in range(len(TopTwo[0])):
          if TopTwo[0][i][0][0] < X1:
            X1 = TopTwo[0][i][0][0]

      X2 = 0

      if (len(TopTwo)>1):
        for i in range(len(TopTwo[1])):
          if TopTwo[1][i][0][0] > X2:
            X2 = TopTwo[1][i][0][0]

      Y1 = 10000

      if (len(TopTwo)>1):
        for i in range(len(TopTwo[0])):
          if abs(TopTwo[0][i][0][0]-X1) < 10:
            if TopTwo[0][i][0][1] < Y1:
              Y1 = TopTwo[0][i][0][1]

      Y2 = 0

      if (len(TopTwo)>1):
        for i in range(len(TopTwo[1])):
          if abs(TopTwo[1][i][0][0]-X2) < 10:
            if TopTwo[1][i][0][1] > Y2:
              Y2 = TopTwo[1][i][0][1]

      Y3 = 10000
      X3 = X2

      if (len(TopTwo)>1):
        for i in range(len(TopTwo[0])):
          if abs(TopTwo[0][i][0][0]-X3) < 15:
            if TopTwo[0][i][0][1] < Y3:
              Y3 = TopTwo[0][i][0][1]

      Y4 = 0
      X4 = X1

      if (len(TopTwo)>1):
        for i in range(len(TopTwo[1])):
          if abs(TopTwo[1][i][0][0]-X4) < 20:
            if TopTwo[1][i][0][1] > Y4:
              Y4 = TopTwo[1][i][0][1]

      
      TopLeft = (X1, Y1)
      BottomRight = (X2, Y2)
      TopRight = (X3, Y3)
      BottomLeft = (X4, Y4)

      width = abs(X2-X1)
      height = abs(Y2-Y1)

      rows, cols, ch = cv_image.shape    
      pts1 = np.float32([(TopLeft), TopRight, BottomLeft, BottomRight])
      pts2 = np.float32([(0,0),(width, 0),(0,height),(width, height)])
      M = cv2.getPerspectiveTransform(pts1,pts2)
      dst = cv2.warpPerspective(cv_image, M, (cols, rows))

      extract = dst[0:height, 0:width]

      rectangle = Y1<Y4 and Y3<Y2 and X1<X2

      if(height > 0 and (abs(width/height-0.85)<0.225) and X1 > 5 and X2 < 1275 and rectangle):


        contour_color = (0, 255, 0)
        contour_thick = 2

        ParkingSpace = extract[int(0.33*height):int(0.68*height), int(width/2):width]
        License = extract[int(0.705*height):int(0.87*height), 0:width]

        img = cv2.cvtColor(img_blur,cv2.COLOR_GRAY2RGB)

        cv2.circle(img, TopLeft, 10, contour_color, contour_thick)
        cv2.circle(img, TopRight, 10, contour_color, contour_thick)
        cv2.circle(img, BottomLeft, 10, contour_color, contour_thick)
        cv2.circle(img, BottomRight, 10, contour_color, contour_thick)

        cv2.imshow("BlurWhite", img)
        cv2.waitKey(1)

        # divided = cv2.line(extract, (0, int(0.33*height)), (width, int(0.33*height)), contour_color, contour_thick)
        # divided = cv2.line(extract, (0, int(0.68*height)), (width, int(0.68*height)), contour_color, contour_thick)
        # divided = cv2.line(extract, (0, int(0.88*height)), (width, int(0.88*height)), contour_color, contour_thick) 

        if (extract.size>50000 and extract.size<2000000):
          cv2.imshow("Extract", extract)
          cv2.waitKey(1)


          hsvLicense = cv2.cvtColor(License, cv2.COLOR_BGR2HSV)

          uh = 122
          us = 255
          uv = 255
          lh = 118
          ls = 80
          lv = 0
          lower_hsv = np.array([lh,ls,lv])
          upper_hsv = np.array([uh,us,uv])
        
          WhiteLicense = cv2.inRange(hsvLicense, lower_hsv, upper_hsv)

          hsvParking = cv2.cvtColor(ParkingSpace, cv2.COLOR_BGR2HSV)

          uh = 5
          us = 5
          uv = 50
          lh = 0
          ls = 0
          lv = 0
          lower_hsv = np.array([lh,ls,lv])
          upper_hsv = np.array([uh,us,uv])
        
          hsvParking = cv2.inRange(hsvParking, lower_hsv, upper_hsv)


          WhiteLicense = cv2.resize(WhiteLicense,(600,300))
          hsvParking = cv2.resize(hsvParking,(100,140))

          if (ParkingSpace.size>0):
            cv2.imshow("ParkingSpace", hsvParking)
            cv2.waitKey(1)

          if (License.size>0):
            cv2.imshow("License", WhiteLicense)
            cv2.waitKey(1)

        rect = cv2.rectangle(cv_image, (X1, Y1), (X2, Y2), contour_color, contour_thick)

        cv2.imshow("Contours", rect)
        cv2.waitKey(1)
      else:
        cv2.imshow("Contours", cv_image)
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
