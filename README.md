# ENPH353_CompetitionRepo2

The controller_pkg repository contains the scripts that execute the robot's desired functionality when launched. This is a ROS package containing:

- Control algorithm for the robot in scripts CompTimer.py, LicensePlateDetection.py, line_follow.py, and ped_detct.py.
- The launch file competition.launch that launches the above scripts.
- A convolutional neural network model file trained from ENPH353_CompetitionRepo3 (my_model.h5), used in LicensePlateDetection.py.
