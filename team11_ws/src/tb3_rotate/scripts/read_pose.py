#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class pose_read:
    def __init__(self):
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.callback)
        self.cam_sub = rospy.Subscriber("/svo/pose", PoseWithCovarianceStamped, self.callback2)

    def callback(self, imu_msg):
        quaternion = (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        print "IMU: Roll: %s Pitch: %s Yaw: %s (rad)" % (roll, pitch, yaw)

    def callback2(self, msg):
        quaternion2 = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll2, pitch2, yaw2) = euler_from_quaternion(quaternion2)
        print "CAM: Roll: %s Pitch: %s Yaw: %s (rad)" % (roll2, pitch2, yaw2)

if __name__ == '__main__':
    rospy.init_node('read_pose', anonymous=True)
    pose_data = pose_read()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
