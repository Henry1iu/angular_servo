#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# import lib
import time
import numpy as np

import cv2 

# import ros lib
import rospy, cv_bridge

from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from std_srvs.srv import SetBool

from lib.pid.pid import PID


ROS_RATE = 30 	# in Hz
DEFAULT_FORWARD_CYCLE = ROS_RATE * 3
MIN_MATCHED_FEATURE = 20
KP = 0.5

KD = 0.001
DEFAULT_FORWARD_SPEED_1 = - 0.05
DEFAULT_FORWARD_SPEED_2 = - 0.15
MAXIMUM_ROTATE_SPEED = 0.08

TARGET_POINT = (900, 730)		# x(width), y(height)

DRAW_MATCHES = True

GT_IMG_PATH = "/home/rpai_asrock/bonobot_ws/src/angular_servo/data/ref.jpg"
DISPLAY_WINDOW_NAME = "Angular Servo"


class AngleServo(object):

	def __init__(self):
		self.__node = rospy.init_node("task2_angular_servo")
		self.__rate = rospy.Rate(ROS_RATE)

		# subscriber
		self.__img_sub = rospy.Subscriber("/usb_cam/raw_img", Image, self.__img_callback__, queue_size=1, buff_size=6220800)
		self.__ctr_sub = rospy.Subscriber("/angular_servo/control", Int8, self.__start_callback__)
		self.__dock_stage = rospy.Subscriber("/dockStage", Int8, self.__dock_callback__)

		# publisher
		self.__rot_pub = rospy.Publisher("/yocs_cmd_vel_mux/servoing/cmd_vel",Twist, queue_size=1)

		# srv
		self.__stage_2_done_srv = rospy.ServiceProxy("/dockStage2DoneSrv", SetBool)                                                

		# tag
		self.__activated = False
		self.__rotating = False
		self.__forwarding = False

		# variable
		self.__foward_cycle = DEFAULT_FORWARD_CYCLE

		self.__bridge = cv_bridge.CvBridge()
		self.__orb = cv2.ORB_create(edgeThreshold=25, 
								    scoreType=cv2.ORB_FAST_SCORE, 
							  		WTA_K=2, 
								  	nfeatures=2000)
		self.__bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
		
		self.__gt_img = cv2.imread(GT_IMG_PATH, 0)[620:840, 600:1200]
		self.__img_h, self.__img_w = self.__gt_img.shape
		self.__gt_kp, self.__gt_des = self.__orb.detectAndCompute(self.__gt_img, None)

		self.__cnt_point_valid = False
		self.__cnt_point = None

		# controller
		self.__target_point = TARGET_POINT
		self.__pid = PID(kp=KP,
                       	 ki=0.0,
                       	 kd=KD,
                       	 deadband=0.00,
                       	 u_min=-1.0,
                       	 u_max=1.0,
                         e_int_min=0.0,
                         e_int_max=0.0,
                         dt=0.1)

		# display component
		self.__disp_img = None
		self.__window = cv2.namedWindow(DISPLAY_WINDOW_NAME, cv2.WINDOW_NORMAL)
		# cv2.resizeWindow(DISPLAY_WINDOW_NAME, 640, 480)

	# def __reset__(self):
	# 	self.__start = False
	# 	self.__reached = False

	def __dock_callback__(self, msg):
		if not self.__activated:
			self.__activated = True
		else:
			if msg.data == 2 and not self.__rotating and not self.__forwarding:
				self.__rotating = True


	def __start_callback__(self, msg):
		if msg.data == 1:
			self.__activated = True
		elif msg.data == 2:
			self.__rotating = True
		elif msg.data == 3:
			self.__capture_reference__()
		elif msg.data == 4:
			self.__activated = False
		elif msg.data == 5:
			self.__rotating = False

	def __img_callback__(self, msg):
		# Do not process the img msg if not activated
		if not self.__activated:
			return

		img = self.__bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		if self.__rotating:								# detect and match features if rotating
			kp, des = self.__orb.detectAndCompute(img, None)
		
			if len(kp) != 0:
				matches = self.__bf.match(self.__gt_des, des)
				src_pts = np.float32([ self.__gt_kp[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
				dst_pts = np.float32([ kp[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)

				M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
				matchesMask = mask.ravel().tolist()

				good_matches = [matches[i] for i in range(len(mask)) if mask[i][0] == 1]
				# for i in range(len(mask)):
				# 	if mask[i][0] == 1:
				# 	    good_matches.append(matches[i])
				sorted_good_matches = sorted(good_matches, key=lambda x:x.distance)

				perfect_src_pts = np.float32([ self.__gt_kp[m.queryIdx].pt for m in sorted_good_matches ]).reshape(-1,1,2)
				perfect_dst_pts = np.float32([ kp[m.trainIdx].pt for m in sorted_good_matches ]).reshape(-1,1,2)

				try:
					M2, mask2 = cv2.findHomography(perfect_src_pts, perfect_dst_pts, cv2.RANSAC,5.0)
					matchesMask2 = mask2.ravel().tolist()
				except:
					sorted_good_matches = matches
				else:
					M = M2
					matchesMask = matchesMask2

				if len(matchesMask) >= MIN_MATCHED_FEATURE:
					center_ref = np.float32([[(self.__img_w-1)/2, (self.__img_h-1)/2]]).reshape(-1, 1, 2)
					self.__cnt_point = cv2.perspectiveTransform(center_ref, M)
					self.__cnt_point_valid = True

					# plot the bbox and reference box center and current box center
					bbox = np.float32([[0, 0], 
							           [0, self.__img_h - 1], 
							           [self.__img_w - 1, self.__img_h - 1], 
							           [self.__img_w - 1, 0]]).reshape(-1, 1, 2)
					bbox = cv2.perspectiveTransform(bbox, M)
					img = cv2.polylines(img, [np.int32(bbox)], True, 255, 5, cv2.LINE_AA)
					img = cv2.circle(img, (np.int32(self.__cnt_point[0,0,0]), np.int32(self.__cnt_point[0,0,1])), 10, (0,0,255), 4)
					img = cv2.circle(img, (np.int32(self.__target_point[0]), np.int32(self.__target_point[1])), 10, (0,255,0), 4)

					if DRAW_MATCHES:
						draw_params = dict(matchColor=(0, 255, 0),
										   singlePointColor=None,
										   matchesMask=matchesMask,
										   flags=2)
						img = cv2.drawMatches(self.__gt_img, self.__gt_kp, img, kp, sorted_good_matches, None, **draw_params)
				else:
					print("Info: No enough feature point!")

		self.__disp_img = img

	def __rotate__(self):
		# define the twist
		if not self.__cnt_point_valid:
			print("Info: Control point is not valid! Skip this cycle...")
			return

		if self.__cnt_point[0, 0, 1] >= self.__target_point[1]:
			self.__cnt_point_valid = False			
			self.__rotating = False
			self.__forwarding = True
			print("Info: the target point has been reached! Continue to move forward for 2 seconds...")
			return

		self.__cnt_point_valid = False
		distance = self.__cnt_point[0, 0, 0] - self.__target_point[0]

		twist = Twist()
		twist.linear.x = DEFAULT_FORWARD_SPEED_1
		twist.angular.z = min(self.__pid.PID_CalcOutput(distance/1920), MAXIMUM_ROTATE_SPEED)

		self.__rot_pub.publish(twist)
		print("Info: New rotation twist has been published! Z: {}".format(twist.angular.z))

	def __move_forward__(self, speed=DEFAULT_FORWARD_SPEED_2):
		if self.__foward_cycle != 0:
			twist = Twist()

			twist.linear.x = speed
			self.__rot_pub.publish(twist)
			self.__foward_cycle = self.__foward_cycle - 1
		else:
			self.__stage_2_done_srv(True)
			time.sleep(1)
			self.__forwarding = False
			self.__foward_cycle = DEFAULT_FORWARD_CYCLE
			print("Info: Reached the final destiantion! Feed me next target...")

	# def __move_backward__(self, speed=0.1):
	# 	twist = Twist()

	# 	twist.linear.x = -abs(speed)
	# 	self.__rot_pub.publish(twist)

	def __capture_reference__(self):
		if self.__rotating or self.__forwarding:
			print("ERROR: Can't capture ground truth image while the robot is moving!")
			return

		# wait the current img to be valid
		while self.__disp_img is None:
			continue

		self.__gt_img = self.__disp_img[680:880, 620:1250]
		self.__gt_kp, self.__gt_des = self.__orb.detectAndCompute(self.__gt_img, None)
		print("Info: New ground truth image has been captured!!!")

	def run(self):
		while not rospy.is_shutdown():
			if self.__rotating == True:
				self.__rotate__()
			elif self.__forwarding == True:
				self.__move_forward__()

			# display the current image
			if self.__disp_img is not None:
				cv2.imshow(DISPLAY_WINDOW_NAME, self.__disp_img)
				cv2.waitKey(10)
				self.__disp_img = None
			self.__rate.sleep()


if __name__ == "__main__":
	servo = AngleServo()
	try:
		servo.run()
	except rospy.ROSInterruptException:
		pass
