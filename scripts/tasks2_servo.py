#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# import lib
import numpy as np

import cv2 

# import ros lib
import rospy, cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from lib.pid.pid import PID


ROS_RATE = 10 	# in Hz

PKG_ROOT_DIR = "/home/rpai_asrock/bonobot_ws/src/angular_servo"
GROUND_TRUTH_IMG_NAME = "ref"
GROUND_TRUTH_IMG_FORMAT = "jpg"

class AngleServo(object):
	def __init_(self)
		self.__node__ = rospy.init_node("task2_angular_servo")
		self.__rate__ = rospy.Rate(ROS_RATE)

		# subscriber
		self.__img_sub__ = rospy.Subscriber()
		self.__ctr_sub__ = rospy.Subscriber()

		# publisher
		self.__rot_pub__ = rospy.Publisher()
		self.__finish_pub__ = rospy.Publisher()

		# tag
		self.__start__ = False
		self.__reached__ = False

		# variable
		bridge = cv_bridge.CvBridge()
		self.__orb__ = cv2.ORB_create(edgeThreshold=25, scoreType=cv2.ORB_FAST_SCORE, WTA_K=2, nfeatures=2000)
		self.__bf__ = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
		
		self.__gt_img__ = cv2.imread(os.path.join(PKG_ROOT_DIR, "data", "{}.{}".format(GROUND_TRUTH_IMG_NAME,
				   							       ROUND_TRUTH_IMG_FORMAT)))
		self.__gt_kp__, self.__gt_des__ = self.__orb__.detectAndCompute(self.__gt_img__, None)

		self.__cnt_img__ = None
		self.__cnt_kp__ = []
		self.__cnt_des__ = []

		# controller
		self.__pid__ = PID(kp=0.07,
                           ki=0.0,
                           kd=0.001,
                           deadband=0.00,
                           u_min=-1.0,
                           u_max=1.0,
                           e_int_min=0.0,
                           e_int_max=0.0,
                           dt=0.1)
		
	def __reset__(self):
		self.__start__ = False
		self.__reached__ = False

	def start_callback(self, msg)
		pass

	def img_callback(self, msg):
		if not self.__start__:
			return

		if self.__reached__:
			return 

		img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		kp, des = orb.detectAndCompute(img, None)

		if len(kp) == 0:
			return
		matches = self.__bf__.match(self.__gt_kp__, kp）

		if ()

	def run(self)
		while not rospy.is_shutdown():
			self.__rate__.sleep() 

	def rotate(self, angle)
		# define the twist
		twist = Twist()

		twist.linear.x = 0.1
		twist.linear.y = 0
		twist.linear.z = 0

		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = self.__pid__.PID_CalcOutput(angular)

		self.__rot_pub__.publish(twist)

	def compute_angle(self, img)

if __name__ == "__main__":
	servo = AngleServo()
	try:
		servo.run()
	except rospy.ROSInterruptException:
		pass
