#!/usr/bin/env python3

from __future__ import print_function

import roslib
import rospy
import sys
from geometry_msgs.msg import Twist
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import time

rospy.init_node('TimeTrialsTimer', anonymous=True)

pub = rospy.Publisher('/license_plate', String, queue_size=1)
time.sleep(30)

message = str('TeamRed,multi21,-1,FE08')

pub.publish(message)

