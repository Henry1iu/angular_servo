#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import numpy as np


MIN_MATCHED_FEATURE = 30

# img1 = cv2.imread() [600:1200, 800:1000, :]
# img2 = cv2.imread("/home/henry/catkin_ws/src/angular_servo/data/ref.jpg", 0)
# img2_crop = cv2.imread("/home/henry/catkin_ws/src/angular_servo/data/ref.jpg", 0)[800:1000, 600:1200]
# orb detector
orb = cv2.ORB_create(edgeThreshold=25, scoreType=cv2.ORB_FAST_SCORE, WTA_K=2, nfeatures=2000)
# sift detector
# detector = cv2.xfeatures2d.SURF_create(hessianThreshold=400)

# bf matcher
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
# FLANN matcher
index_params= dict(algorithm = 6,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2
search_params = dict(checks=50)   # or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params,search_params)


cv2.namedWindow("output", cv2.WINDOW_AUTOSIZE)
# kp2, des2 = orb.detectAndCompute(img2, None)
# kp2_crop, des2_crop = orb.detectAndCompute(img2_crop, None)

# img2_out = img2
# img2_out = cv2.drawKeypoints(img2, kp2, img2_out, color=(0,255,0), flags=0)

# img2_crop_out = img2_crop
# img2_crop_out = cv2.drawKeypoints(img2_crop, kp2_crop, img2_crop_out, color=(0,255,0), flags=0)

# cv2.namedWindow("output", cv.WINDOW_AUTOSIZE)
# cv2.imshow("output", img2_out)
# cv2.waitKey(0)

# cv2.imwrite("/home/henry/catkin_ws/src/angular_servo/data/ref_detected.jpg", img2_out)
# cv2.imwrite("/home/henry/catkin_ws/src/angular_servo/data/ref_crop_detected.jpg", img2_crop_out)

for i in range(454):
	# ref = cv2.imread("/home/henry/projects/airportTrolleyCollection/task2/rgbServo/img/alipay2.jpg", 0)
	ref = cv2.imread("/home/henry/catkin_ws/src/angular_servo/data/ref.jpg", 0)[800:-50, 500:-500]
	
	img = cv2.imread("/home/henry/catkin_ws/src/angular_servo/data/imgs/%d.jpg" % i, 0)[200:-1, 200:-200]

	kp_ref, des_ref = orb.detectAndCompute(ref, None)
	kp, des = orb.detectAndCompute(img, None)

	matches = bf.match(des_ref, des)
	matches = sorted(matches, key = lambda x:x.distance)

	# matches = flann.knnMatch(des_ref,des,k=2)
	# good = []
	# for m,n in matches:
	# 	if m.distance < 0.7*n.distance:
	# 	    good.append(m)
	# matches = good


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
	good_src_pts = np.float32(good_src_pts).reshape(-1,1,2)
	good_dst_pts = np.float32(good_dst_pts).reshape(-1,1,2)

	sorted_good_matches = sorted(good_matches, key=lambda x:x.distance)
	# perfect_matches = sorted_good_matches[:int(count*2/4)]

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

	if len(matchesMask) > MIN_MATCHED_FEATURE:
		h, w = ref.shape
		pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
		dst = cv2.perspectiveTransform(pts, M)

		center_ref = np.float32([[(w-1)/2, (h-1)/2]]).reshape(-1, 1, 2)
		center = cv2.perspectiveTransform(center_ref, M)

		img = cv2.polylines(img, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
		img = cv2.circle(img, (np.int32(center[0,0,0]), np.int32(center[0,0,1])), 5, (255,0,0), 2)

	draw_params = dict(matchColor=(0, 255, 0),
						singlePointColor=None,
						matchesMask=matchesMask,
						flags=2)

	# img_matched = cv2.drawMatches(ref,kp_ref,img,kp, sorted_good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
	# img_matched = cv2.drawMatches(ref,kp_ref,img,kp, sorted_good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
	img_matched = cv2.drawMatches(ref, kp_ref, img, kp, sorted_good_matches, None, **draw_params)

	# img_out = img
	# img_out = cv2.drawKeypoints(img, kp, img_out, color=(0,255,0), flags=0)
	
	cv2.imshow("output", img_matched)
	cv2.waitKey(0)

	# cv2.imwrite("/home/henry/catkin_ws/src/angular_servo/data/detected/%d.jpg" % i, img_out)


