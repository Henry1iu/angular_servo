#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import threading
import numpy as np
import cv2, cv_bridge

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from lib.pid.pid import PID 

ROS_RATE = 15 	# in Hz
MIN_MATCHED_FEATURE = 30

TARGET_POINT = (910, 960) 


tag_start = True
tag_reach = False
tag_new_img = False

reach_countdown = int(2*ROS_RATE)

msg_lock = threading.Lock()

bridge = cv_bridge.CvBridge()
orb = cv2.ORB_create(edgeThreshold=25, scoreType=cv2.ORB_FAST_SCORE, WTA_K=2, nfeatures=2000)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

ref = cv2.imread("/home/henry/catkin_ws/src/angular_servo/data/ref.jpg", 0)[800:-50, 500:-500]
kp_ref, des_ref = orb.detectAndCompute(ref, None)
h, w = ref.shape

img = None

pid = PID(kp=0.003,
	      ki=0.0,
	      kd=0.001,
	      deadband=0.00,
	      u_min=-1.0,
	      u_max=1.0,
	      e_int_min=0.0,
	      e_int_max=0.0,
	      dt=0.1)

def img_callback(msg):
	global tag_start, tag_reach, img, bridge, msg_lock, tag_new_img
	# if msg_lock.acquire(False):
	# print("Received image messages!")
	if True:
		if not tag_start:
			return

		if tag_reach:
			return

		try:
			img = bridge.imgmsg_to_cv2(msg, "bgr8")
			tag_new_img = True
			print("Received new image!")
		except cv_bridge.CvBridgeError as e:
			return
		# msg_lock.release()

node = rospy.init_node("task2_angular_servo")
rate = rospy.Rate(ROS_RATE)

img_sub = rospy.Subscriber("/usb_cam/raw_img", Image, img_callback)

rot_pub = rospy.Publisher("/yocs_cmd_vel_mux/servoing/cmd_vel", Twist, queue_size=30)

while not rospy.is_shutdown():
	# do nothing when not start
	if not tag_start:
		continue

	# when reach the point, move 30 cycles further and stop
	if tag_reach:
		if reach_countdown != 0:
			twist = Twist()
			twist.linear.x = 0.1
			twist.linear.y = 0
			twist.linear.z = 0
			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = 0

			reach_countdown = reach_countdown - 1
		else:
			print("Reach the final position!")
			break
		continue

	# process the img only once
	# if msg_lock.acquire(False) and tag_new_img:
	if tag_new_img:
		kp, des = orb.detectAndCompute(img, None)
		# msg_lock.release()

		matches = bf.match(des_ref, des)
		matches = sorted(matches, key = lambda x:x.distance)

		src_pts = np.float32([ kp_ref[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
		dst_pts = np.float32([ kp[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)

		M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
		matchesMask = mask.ravel().tolist()

		good_matches = []
		good_src_pts = []
		good_dst_pts = []
		count = 0
		for i in range(len(mask)):
			if mask[i][0] == 1:
			    count += 1
			    good_matches.append(matches[i])
			    good_src_pts.append(kp_ref[matches[i].queryIdx].pt)
			    good_dst_pts.append(kp[matches[i].trainIdx].pt)

		sorted_good_matches = sorted(good_matches, key=lambda x:x.distance)

		perfect_src_pts = np.float32([ kp_ref[m.queryIdx].pt for m in sorted_good_matches ]).reshape(-1,1,2)
		perfect_dst_pts = np.float32([ kp[m.trainIdx].pt for m in sorted_good_matches ]).reshape(-1,1,2)

		try:
			M2, mask2 = cv2.findHomography(perfect_src_pts, perfect_dst_pts, cv2.RANSAC,5.0)
			matchesMask2 = mask2.ravel().tolist()
		except:
			sorted_good_matches = matches
		else:
			M = M2
			matchesMask = matchesMask2

		if len(matchesMask) < MIN_MATCHED_FEATURE:
			print("No enough feature point!")
			continue

		center_ref = np.float32([[(w-1)/2, (h-1)/2]]).reshape(-1, 1, 2)
		center = cv2.perspectiveTransform(center_ref, M)

		if center[0,0,1] > TARGET_POINT[0]:
			tag_reach = True
			print("Reach the target point! Center: ({}, {})".format(center[0,0,0], center[0,0,1]))
			continue

		distance = center[0,0,0] - TARGET_POINT[1]
		twist = Twist()

		twist.linear.x = 0.1
		twist.linear.y = 0
		twist.linear.z = 0

		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = pid.PID_CalcOutput(distance)

		rot_pub.publish(twist)
		tag_new_img = False
		print("Publish rotation twist: %f" % twist.angular.z)

	else:
		# print("No new image received!")
		continue

	rate.sleep()
