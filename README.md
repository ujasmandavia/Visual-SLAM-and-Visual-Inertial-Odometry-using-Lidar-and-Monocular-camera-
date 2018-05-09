# Visual SLAM and Visual Inertial Odometry using Lidar and Monocular camera

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

INSTALL INSTRUCTIONS ON TURTLEBOT3
https://github.com/ROBOTIS-GIT/turtlebot3
# setup the Gstreamer (640x480, 30fps)
https://discourse.ros.org/t/ros-visual-odometry/2465

INSTALL INSTRUCTIONS ON LOCAL MACHINE
copy team11_ws to home directory
cd team11_ws
catkin_make
# if an error is encountered, please install the ros dependency package mentioned
source devel/setup.bash
# don't forget to use "chmod" to grant permission to all python scripts

USAGE
Face Follower Demo
TURTLEBOT3
start "bringup" and Gstreamer on Turtlebot3
LOCAL
# open a new terminal
roslaunch gscam raspicam.launch
# open a new terminal
cd ~/team11_ws/src/tb3_rotate/scripts/
rosrun tb3_rotate test_follow.py


Monocular Visual Odometry Demo
TURTLEBOT3
start "bringup" and Gstreamer on Turtlebot3
LOCAL
# open a new terminal
roslaunch turtlebot3_slam turtlebot3_slam.launch
# open a new terminal
roslaunch gscam raspicam.launch
# open a new terminal
roslaunch team11_opticalflow main.launch
# then use teleoperation to control the turtlebot
# or use the frontier_exploration package

APPENDIX
Part of the required installation package
gscam
image_common
image_pipeline
navigation
robot_localization
vision_opencv
viso2
