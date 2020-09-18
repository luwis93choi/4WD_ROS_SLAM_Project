# Reference 01 (Optical Flow-based Visual Odometry) : https://apollack11.github.io/advanced-computer-vision.html
# Reference 02 (Optical Flow and Camera Velocity) : https://robotacademy.net.au/lesson/optical-flow-and-camera-velocity/
# Reference 03 (OpenCv Optical Flow Example) : https://docs.opencv.org/3.3.1/d7/d8b/tutorial_py_lucas_kanade.html

import cv2 as cv
import numpy as np
import os
import sys

import pyrealsense2 as rs

print("OpenCV Ver : " + cv.__version__)

pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

# Params for ShiTomasi Corner Detection
feature_params = dict(maxCorners = 100,
                      qualityLevel = 0.3,
                      minDistance = 7,
                      blockSize = 7)

# Params for Lucas-Kanade Optical Flow
lk_params = dict(winSize = (21, 21),
                 maxLevel = 2,
                 criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 30, 0.01))

# Camera intrinsic matrix params for 2D-3D triangulation
intrinsicCamMat = np.array([[1.93, 0, rs.intrinsics().ppx, 0],
                            [0, 1.93, rs.intrinsics().ppy, 0],
                            [0, 0, 1, 0]])

print(rs.intrinsics())
print(rs.extrinsics())

# Create some random colors
color = np.random.randint(0, 255, (100, 3))

### 1st image frame ##############################################################################
# Take the first frame and find corners in it
frames = pipeline.wait_for_frames()
prev_frames = frames.get_color_frame()

prev_image = np.asanyarray(prev_frames.get_data())

prev_image_gray = cv.cvtColor(prev_image, cv.COLOR_BGR2GRAY)

p0 = cv.goodFeaturesToTrack(prev_image_gray, mask=None, **feature_params)

# Create a mask image for drawing purposes
mask = np.zeros_like(prev_image)

ref_ready = False

while True:

    ### Acquire 2nd image frame and conduct 3-D triangulation between 1st frame and 2nd frame
    ### This is to acquire reference 3-D triangulation points for relative scaling
    if ref_ready is False:

        frames = pipeline.wait_for_frames()
        current_frames = frames.get_color_frame()

        current_image = np.asanyarray(current_frames.get_data())

        current_image_gray = cv.cvtColor(current_image, cv.COLOR_RGB2GRAY)

        # Calculate Optical Flow
        p1, st, err = cv.calcOpticalFlowPyrLK(prev_image_gray, current_image_gray, p0, None, **lk_params)

        # If optical flow is successful with feature tracking,
        # collect good feature points and draw the tracks on the image
        if (p1 is not None) and (p0 is not None):

            # Feature-based pose estimation requires a certain number of features (5-point or 8-point)
            # If there are not enough features in the image, skip the current image
            if len(p1) >= 8:
                # Select good feature points to track with optical flow
                good_pts_current = p1[st==1]
                good_pts_prev = p0[st==1]

                Essential_Mat, e_mask = cv.findEssentialMat(good_pts_prev, good_pts_current, 
                                                            focal=1.93, pp=(rs.intrinsics().ppx, rs.intrinsics().ppy),
                                                            method=cv.RANSAC, prob=0.999, threshold=1.0, mask=cv.UMat())

                retval, Rotation_Mat, Translation_Mat, newMask = cv.recoverPose(Essential_Mat, good_pts_prev, good_pts_current, focal=1.93, pp=(rs.intrinsics().ppx, rs.intrinsics().ppy))
                
                '''
                print('Rotation Matrix ')
                print(Rotation_Mat)
                print('Translation Matrix ')
                print(Translation_Mat)
                '''

                extrinsicCamMat = np.hstack((Rotation_Mat, Translation_Mat))

                '''
                print(intrinsicCamMat)
                print(extrinsicCamMat)

                print('good_pts_current shape : ', good_pts_current.shape)
                print('good_pts_prev shape : ', good_pts_prev.shape)
                '''

                good_pts_current_tri = good_pts_current.reshape(2, -1)
                good_pts_prev_tri = good_pts_prev.reshape(2, -1)

                prev_cloud = cv.triangulatePoints(intrinsicCamMat, extrinsicCamMat, good_pts_current_tri, good_pts_prev_tri)

                # Draw the tracks #############################################################
                for i, (current, prev) in enumerate(zip(good_pts_current, good_pts_prev)):
                    a, b = current.ravel()
                    c, d = prev.ravel()
                    mask = cv.line(mask, (a, b), (c, d), color[i].tolist(), 2)
                    current_image = cv.circle(current_image, (a, b), 5, color[i].tolist(), -1)

                img = cv.add(current_image, mask)

                cv.imshow('Optical Flow Feature Tracking', img)

                k = cv.waitKey(30) & 0xff
                if k == 27:
                    break
                ###############################################################################

                # Update the previous frame and previous points
                prev_image_gray = current_image_gray.copy()
                p0 = good_pts_current.reshape(-1, 1, 2)

                ref_ready = True    # With reference cloud points acquired, move onto main processing step

            else:

                prev_image_gray = current_image_gray.copy()
                p0 = cv.goodFeaturesToTrack(prev_image_gray, mask=None, **feature_params)

                # Create a mask image for drawing purposes
                mask = np.zeros_like(prev_image)

        # If optical flow fails to track features,
        # reset the previous image with current image and re-collect features for tracking
        else:

            prev_image_gray = current_image_gray.copy()
            p0 = cv.goodFeaturesToTrack(prev_image_gray, mask=None, **feature_params)

            # Create a mask image for drawing purposes
            mask = np.zeros_like(prev_image)

    elif ref_ready is True:

        frames = pipeline.wait_for_frames()
        current_frames = frames.get_color_frame()

        current_image = np.asanyarray(current_frames.get_data())

        current_image_gray = cv.cvtColor(current_image, cv.COLOR_RGB2GRAY)

        # Calculate Optical Flow
        p1, st, err = cv.calcOpticalFlowPyrLK(prev_image_gray, current_image_gray, p0, None, **lk_params)

        # If optical flow is successful with feature tracking,
        # collect good feature points and draw the tracks on the image
        if (p1 is not None) and (p0 is not None):

            # Feature-based pose estimation requires a certain number of features (5-point or 8-point)
            # If there are not enough features in the image, skip the current image
            if len(p1) >= 8:
                # Select good feature points to track with optical flow
                good_pts_current = p1[st==1]
                good_pts_prev = p0[st==1]

                # Rotation and Translation matrix calculation using Essential matrix
                Essential_Mat, e_mask = cv.findEssentialMat(good_pts_prev, good_pts_current, 
                                                            focal=1.93, pp=(rs.intrinsics().ppx, rs.intrinsics().ppy),
                                                            method=cv.RANSAC, prob=0.999, threshold=1.0, mask=cv.UMat())

                retval, Rotation_Mat, Translation_Mat, newMask = cv.recoverPose(Essential_Mat, good_pts_prev, good_pts_current, focal=1.93, pp=(rs.intrinsics().ppx, rs.intrinsics().ppy))
                
                # 2D-3D Triangulation using intrinsic params, extrinsic params (Transformation Matrix), and tracking features
                # Acquire 3-D points of tracking features in the world with triangulation
                '''
                print('Rotation Matrix ')
                print(Rotation_Mat)
                print('Translation Matrix ')
                print(Translation_Mat)
                '''

                extrinsicCamMat = np.hstack((Rotation_Mat, Translation_Mat))

                '''
                print(intrinsicCamMat)
                print(extrinsicCamMat)

                print('good_pts_current shape : ', good_pts_current.shape)
                print('good_pts_prev shape : ', good_pts_prev.shape)
                '''

                good_pts_current_tri = good_pts_current.reshape(2, -1)
                good_pts_prev_tri = good_pts_prev.reshape(2, -1)

                current_cloud = cv.triangulatePoints(intrinsicCamMat, extrinsicCamMat, good_pts_current_tri, good_pts_prev_tri).reshape(-1, 4)[:, :3]

                # Relative scale computation using the distance between 3-D points of tracking features
                print('prev_cloud shape : ', prev_cloud.shape)
                print('current_cloud shape : ', current_cloud.shape)

                # Between two 3-D points of tracking features from t image, t-1 image, t-2 image, 
                # select the common 3-D points
                common_cloud_num = min([prev_cloud.shape[0], current_cloud.shape[0]])
                print(common_cloud_num)

                # Process common 3-D points for relative scaling
                ratios = []
                for i in range(common_cloud_num):
                    if i > 0:   # Pick a pair of common 3-D points and use them to calculate the depth (disparsity)
                        current_Xk = current_cloud[i]

                # Draw the tracks #############################################################
                for i, (current, prev) in enumerate(zip(good_pts_current, good_pts_prev)):
                    a, b = current.ravel()
                    c, d = prev.ravel()
                    mask = cv.line(mask, (a, b), (c, d), color[i].tolist(), 2)
                    current_image = cv.circle(current_image, (a, b), 5, color[i].tolist(), -1)

                img = cv.add(current_image, mask)

                cv.imshow('Optical Flow Feature Tracking', img)

                k = cv.waitKey(30) & 0xff
                if k == 27:
                    break
                ###############################################################################

                # Update the previous frame and previous points
                prev_image_gray = current_image_gray.copy()
                p0 = good_pts_current.reshape(-1, 1, 2)

                prev_cloud = current_cloud
                ref_ready = True    # With reference cloud points acquired, move onto main processing step

            else:

                prev_image_gray = current_image_gray.copy()
                p0 = cv.goodFeaturesToTrack(prev_image_gray, mask=None, **feature_params)

                # Create a mask image for drawing purposes
                mask = np.zeros_like(prev_image)

        # If optical flow fails to track features,
        # reset the previous image with current image and re-collect features for tracking
        else:

            prev_image_gray = current_image_gray.copy()
            p0 = cv.goodFeaturesToTrack(prev_image_gray, mask=None, **feature_params)

            # Create a mask image for drawing purposes
            mask = np.zeros_like(prev_image)



cv.destroyAllWindows()