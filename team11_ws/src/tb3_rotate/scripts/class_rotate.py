#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class tb3_rotate:
    def __init__(self, publisher):
        self.ang_pub = publisher
        self.is_rotate = False
        self.yaw_init = self.get_angle()

    def vels(self, target_angular_vel):
        return "currently:\tangular vel %s" % (target_angular_vel)

    def pub_ang_speed(self, control_angular_vel):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = control_angular_vel
        if control_angular_vel == 0:
            print "stopped!"
        else:
            print self.vels(control_angular_vel)
        self.ang_pub.publish(twist)

    # rotate at certain angular speed
    def rotate(self, ang_speed):
        target_angular_vel = ang_speed
        control_angular_vel = 0.0
        try:
            # maximum angular speed
            if abs(ang_speed) > 1:
                target_angular_vel = 0
            if target_angular_vel == 0:
                self.pub_ang_speed(0)
            elif target_angular_vel > 0:
                while True:
                    if target_angular_vel > control_angular_vel:
                        control_angular_vel = min(target_angular_vel, control_angular_vel + (0.1 / 4.0))
                        self.pub_ang_speed(control_angular_vel)
                    else:
                        control_angular_vel = target_angular_vel
                        self.pub_ang_speed(control_angular_vel)
                        break
            else:
                while True:
                    if target_angular_vel < control_angular_vel:
                        control_angular_vel = max(target_angular_vel, control_angular_vel - (0.1 / 4.0))
                        self.pub_ang_speed(control_angular_vel)
                    else:
                        control_angular_vel = target_angular_vel
                        self.pub_ang_speed(control_angular_vel)
                        break
        except:
            pass

    # rotate to an angle (-pi ~ pi)
    def receive_imu_and_rotate(self, angle_value):
        imu_msg = rospy.wait_for_message('/imu', Imu)
        quaternion = (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.yaw_init = yaw
        print "rotate to angle: %s, start from: %s" % (angle_value, self.yaw_init)
        while True:
            imu_msg = rospy.wait_for_message('/imu', Imu)
            quaternion = (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
            yaw_prev = yaw
            (roll, pitch, yaw) = euler_from_quaternion(quaternion)
            # rotation start
            if self.is_rotate == False:
                if abs(angle_value - self.yaw_init) < 0.1:
                    self.yaw_init = yaw
                    print "already rotated!"
                    break
                # clockwise
                elif (angle_value - self.yaw_init < -3.14) or (angle_value > self.yaw_init and angle_value - self.yaw_init < 3.14):
                    self.rotate(0.5)
                    self.is_rotate = True
                    print "start rotating!"
                # counter-clockwise
                else:
                    self.rotate(-0.5)
                    self.is_rotate = True
                    print "start rotating!"
            # rotation stop
            else:
                print "currently:\tangle: %s" % (yaw)
                if (yaw_prev<angle_value and yaw>angle_value) or (yaw_prev>angle_value and yaw<angle_value):
                    # when jumping from pi -> -pi or -pi -> pi
                    if abs(yaw-angle_value)>0.8:
                        continue
                    self.rotate(0)
                    self.is_rotate = False
                    self.yaw_init = yaw
                    print "rotation completed!"
                    break

    # get the current yaw angle (-pi ~ pi)
    def get_angle(self):
        imu_msg = rospy.wait_for_message('/imu', Imu)
        quaternion = (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        return yaw

    # rotate an angle
    def rotate_angle(self, angle):
        yaw_init = self.get_angle()
        # make sure angle: -pi ~ pi
        angle_stop = yaw_init + angle
        if angle_stop > 3.14:
            angle_stop = angle_stop - 2*3.14
        elif angle_stop < -3.14:
            angle_stop = angle_stop + 2*3.14
        self.receive_imu_and_rotate(angle_stop)