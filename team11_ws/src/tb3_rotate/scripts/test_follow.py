#!/usr/bin/env python

import sys
import rospy
import cv2
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from class_rotate import tb3_rotate

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_alt2.xml")
        self.ang_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.test_rotate = tb3_rotate(self.ang_pub)
        self.is_locked = False
        self.distance_est = 0.0

    def face_detect_and_rotate(self):
        data = rospy.wait_for_message("/raspicam/image_raw/compressed", CompressedImage)
        print "image received!"
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame_width = cv_image.shape[1]
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(cv_image_gray, 1.3, 4)
        result = []
        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            result.append((x, y, w, h))

        cv2.imshow("image from raspi", cv_image)
        cv2.waitKey(3)

        if self.is_locked == True:
            # if face is already locked, then start following
            if not len(result) == 0:
                face_offset = result[0][0] + result[0][2] / 2.0 - frame_width / 2.0
                angle_offset = math.atan(face_offset / (frame_width / 2.0))
                if abs(angle_offset) < 0.2: # change the coefficient
                    # start moving forward or backward
                    distance_temp = result[0][2] * result[0][3]
                    dist_est = math.sqrt(distance_temp/self.distance_est)
                    if dist_est < 0.9:
                        twist_temp = Twist()
                        twist_temp.linear.x = 0.05
                        twist_temp.linear.y = 0
                        twist_temp.linear.z = 0
                        twist_temp.angular.x = 0
                        twist_temp.angular.y = 0
                        twist_temp.angular.z = 0
                        self.ang_pub.publish(twist_temp)
                    elif dist_est > 1.1:
                        twist_temp = Twist()
                        twist_temp.linear.x = -0.05
                        twist_temp.linear.y = 0
                        twist_temp.linear.z = 0
                        twist_temp.angular.x = 0
                        twist_temp.angular.y = 0
                        twist_temp.angular.z = 0
                        self.ang_pub.publish(twist_temp)
                    else:
                        self.test_rotate.pub_ang_speed(0)
                else:
                    self.is_locked = False
                    self.test_rotate.pub_ang_speed(0)
            # if face not detected, then do nothing
            else:
                self.is_locked = False
                self.test_rotate.pub_ang_speed(0)
        else:
            if not len(result) == 0:
                face_offset = result[0][0] + result[0][2] / 2.0 - frame_width / 2.0
                angle_offset = math.atan(face_offset / (frame_width / 2.0))
                print "face detected! Offset: %s rad" % angle_offset
                # start rotating
                try:
                    self.test_rotate.rotate_angle(-angle_offset * 0.3)
                    self.is_locked = True
                    self.distance_est = result[0][2]*result[0][3]
                    print 'stop at angle %s (rad) and face locked' % (self.test_rotate.get_angle())
                except:
                    pass

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()

    try:
        while True:
            ic.face_detect_and_rotate()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
