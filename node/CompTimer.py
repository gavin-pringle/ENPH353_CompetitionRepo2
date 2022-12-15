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
import os

class timer_converter:

    def __init__(self):
        self.clock_sub = rospy.Subscriber("/clock", String, self.clock_callback, queue_size=1)
        self.score_pub = rospy.Publisher('/license_plate', String, queue_size=1)
        self.drive_pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)

        rospy.sleep(1)
        message = str('Loopy,multi21,0,FE08')
        self.score_pub.publish(message)

    def clock_callback(self, data):
        if "  secs: 110" in str(data):
            os.system("rosnode kill /LicensePlateDetection")
            os.system("rosnode kill /license_server")
            os.system("rosnode kill /ped_detect")
            os.system("rosnode kill /line_follow")
            self.score_pub.publish(str('TeamRed,multi21,-1,FE08'))
            move = Twist()
            move.linear.x = 0
            move.angular.z = 0
            self.drive_pub.publish(move)

## Main function that keeps the simulation running until it is stopped by a user
def main(args):
  # name of node is CompTimer
  rospy.init_node('CompTimer', anonymous=True)
  tc = timer_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)