#!/usr/bin/env python3

from __future__ import print_function

import roslib
import rospy
import sys
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

rospy.init_node('start_up', anonymous=True)

#start the timer
pub = rospy.Publisher('/license_plate', String, queue_size=1)
rospy.sleep(1)

message = str('TeamRed,multi21,0,FE08')

pub.publish(message)

