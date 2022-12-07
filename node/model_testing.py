import numpy as np

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers
from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend
import cv2


def characterFromOneHot(chara):
    characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
    return characters[chara]


my_model = models.load_model("/home/fizzer/ros_ws/src/controller_pkg/node/my_model.h5")

hsvParking =  cv2.imread("/home/fizzer/ros_ws/src/controller_pkg/node/plate_5_1210.png")

hsvParking = np.asarray(hsvParking)


x = np.zeros((1,170,100,3))

x[0] = hsvParking

output=my_model.predict(x)
prediction = characterFromOneHot(np.argmax(output))
print(prediction)







# #!/usr/bin/env python3

# from __future__ import print_function

# import roslib
# import numpy as np
# import sys
# from geometry_msgs.msg import Twist
# import rospy
# import cv2
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import time
# import os
# from tensorflow.keras import layers
# from tensorflow.keras import models
# from tensorflow.keras import optimizers
# from tensorflow.keras.utils import plot_model
# from tensorflow.keras import backend

# def characterFromOneHot(chara):
#     characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
#     return characters[chara]

# my_model = models.load_model("/home/fizzer/ros_ws/src/controller_pkg/node/my_model.h5")

# output = my_model.predict()

# hsvParking =  cv2.imread("/home/fizzer/ros_ws/src/controller_pkg/node/plate_5_1210.png")

# hsvParking = np.asarray(hsvParking)

# prediction = characterFromOneHot(np.argmax(output(hsvParking)))

# print(prediction)

