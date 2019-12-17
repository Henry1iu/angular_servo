#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy, rosbag

import cv2, cv_bridge

from sensor_msgs.msg import Image


target_bag_file = "/home/henry/projects/airportTrolleyCollection/task2/rgbServo/bagfiles/rgb_angular_servo_20191212_2.bag"
time_stamp_captured = 1576141812.312051


cv2.namedWindow("output", cv2.WINDOW_NORMAL)
bridge = cv_bridge.CvBridge()
bag = rosbag.Bag(target_bag_file, "r")
img_msg_list = [msg for _, msg, _ in bag.read_messages(topics=['/usb_cam/raw_img'])]
count = 0

for img_msg in img_msg_list:
	try:
		img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
		print("Find the reference img！")
	except:
		continue
	
	# cv2.imshow("output", img)
	# cv2.waitKey(3)

	cv2.imwrite("/home/henry/catkin_ws/src/angular_servo/data/imgs/%d.jpg" % count, img)
	count = count + 1