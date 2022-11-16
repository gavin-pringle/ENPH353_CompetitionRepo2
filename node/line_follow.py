#!/usr/bin/env python3

from __future__ import print_function

import roslib
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
    self.drive_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback, queue_size=3)

  ## Callback function that can publish velocity commands to the drive topic
  def callback(self,data):
    # convert image to a format compatible with OpenCV
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    # Using OpenCV to process the image from the robot's camera into a thresholded image
    threshold = 75
    _, img_bin = cv2.threshold(cv_image, threshold, 255, cv2.THRESH_BINARY)

    imgWidth = 1280
    imgHeight = 720

    lineHeight = round(imgHeight * 0.9)
    sum = 0
    numBlack = 0
    for i in range(imgWidth):
      (b, g, r) = img_bin[lineHeight, i]
      if r == 255 and g == 255 and b == 255:
        sum += i
        numBlack += 1
    if numBlack == 0:
      numBlack = 1
    # Find the x coordinate of where the line is on the screen

    coord = sum // numBlack
    for i in range(10):
      if coord >= i * 128 and coord <= (i + 1) * 128:
        state[i] = 1
        break
        
    cv2.putText(img=img_bin, text=str(state), org=(20, 20), 
                fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.5, 
                color=(0, 0, 255),thickness=1)
    cv2.imshow("raw", img_bin)
    cv2.waitKey(3)
    
    # publish drive commands
    '''
    try:
      self.drive_pub.publish(move)
    except CvBridgeError as e:
      print(e)
    '''


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
