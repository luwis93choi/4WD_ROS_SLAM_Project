# Reference 01 (Optical Flow-based Visual Odometry) : https://apollack11.github.io/advanced-computer-vision.html
# Reference 02 (Optical Flow and Camera Velocity) : https://robotacademy.net.au/lesson/optical-flow-and-camera-velocity/
# Reference 03 (OpenCv Optical Flow Example) : https://docs.opencv.org/3.3.1/d7/d8b/tutorial_py_lucas_kanade.html

import cv2 as cv
import numpy as np
import os
import sys

import pyrealsense2 as rs

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

print("OpenCV Ver : " + cv.__version__)

pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

# Params for ShiTomasi Corner Detection
feature_params = dict(maxCorners = 500,
                      qualityLevel = 0.3,
                      minDistance = 7,
                      blockSize = 7)

# Params for Lucas-Kanade Optical Flow
lk_params = dict(winSize = (21, 21),
                 maxLevel = 3,
                 criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 30, 0.01))

min_track_features = 8

# Camera intrinsic matrix params for 2D-3D triangulation
# Acquiring Realsense Factory Default Intrinsic Parameters : https://github.com/IntelRealSense/librealsense/issues/2930
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
focal_length = 1.93
fx = intr.fx
fy = intr.fy
cx = intr.ppx
cy = intr.ppy

print('focal length : ', focal_length)
print('fx : ', intr.fx)
print('fy : ', intr.fy)
print('ppx : ', intr.ppx)
print('ppy : ', intr.ppy)

intrinsicCamMat = np.array([[fx, 0, cx],
                            [0, fy, cy],
                            [0, 0,  1]])

pose_T = None
pose_R = None

print(rs.intrinsics())
print(rs.extrinsics())

use_orb = True

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

fig = plt.figure()
plt.grid()

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
        p0, st, err = cv.calcOpticalFlowPyrLK(current_image_gray, prev_image_gray, p1, None, **lk_params)

        # If optical flow is successful with feature tracking,
        # collect good feature points and draw the tracks on the image
        if (p1 is not None) and (p0 is not None):
                
            # Select good feature points to track with optical flow
            good_pts_current = p1
            good_pts_prev = p0

            # Feature-based pose estimation requires a certain number of features (5-point or 8-point)
            # If there are not enough features in the image, skip the current image
            if len(good_pts_current) >= min_track_features:

                Essential_Mat, e_mask = cv.findEssentialMat(good_pts_prev, good_pts_current, 
                                                            focal=focal_length, pp=(cx, cy),
                                                            method=cv.FM_RANSAC, prob=0.999, threshold=1.0, mask=cv.UMat())

                retval, Rotation_Mat, Translation_Mat, newMask = cv.recoverPose(Essential_Mat, good_pts_prev, good_pts_current, focal=focal_length, pp=(cx, cy))

                extrinsicCamMat = np.hstack((Rotation_Mat, Translation_Mat))

                good_pts_current_tri = good_pts_current.reshape(2, -1)
                good_pts_prev_tri = good_pts_prev.reshape(2, -1)

                #prev_cloud = cv.triangulatePoints(intrinsicCamMat, extrinsicCamMat, good_pts_current_tri, good_pts_prev_tri)
                # The canonical matrix (set as the origin)
                P0 = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0]])
                P0 = intrinsicCamMat.dot(P0)
                # Rotated and translated using P0 as the reference point
                P1 = np.hstack((Rotation_Mat, Translation_Mat))
                P1 = intrinsicCamMat.dot(P1)

                prev_cloud = cv.triangulatePoints(P0, P1, good_pts_prev_tri, good_pts_current_tri).reshape(-1, 4)[:, :3]

                # Update the previous frame and previous points
                prev_image_gray = current_image_gray.copy()
                p0 = good_pts_current.reshape(-1, 1, 2)

                pose_T = Translation_Mat
                pose_R = Rotation_Mat

                ref_ready = True    # With reference cloud points acquired, move onto main processing step

                print(pose_T)

                # Draw the trajectory #########################################################
                plt.xlim(-100.0, 100.0)
                plt.ylim(-100.0, 100.0)
                plt.scatter(pose_T[0], pose_T[2])

                plt.draw()
                plt.show(block=False)
                plt.pause(0.0001)
                
                # Draw the tracks #############################################################
                for i, (current, prev) in enumerate(zip(good_pts_current, good_pts_prev)):
                    a, b = current.ravel()
                    c, d = prev.ravel()
                    mask = cv.line(mask, (a, b), (c, d), color[i%100].tolist(), 2)
                    current_image = cv.circle(current_image, (a, b), 5, color[i%100].tolist(), -1)

                img = cv.add(current_image, mask)

                cv.imshow('Optical Flow Feature Tracking', img)

                k = cv.waitKey(30) & 0xff
                if k == 27:
                    break
                ###############################################################################

            else:

                prev_image_gray = current_image_gray.copy()
                p0 = cv.goodFeaturesToTrack(prev_image_gray, mask=None, **feature_params)

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
        p0, st, err = cv.calcOpticalFlowPyrLK(current_image_gray, prev_image_gray, p1, None, **lk_params)

        # If optical flow is successful with feature tracking,
        # collect good feature points and draw the tracks on the image
        if (p1 is not None) and (p0 is not None):
            
            # Select good feature points to track with optical flow
            good_pts_current = p1
            good_pts_prev = p0

            # Feature-based pose estimation requires a certain number of features (5-point or 8-point)
            # If there are not enough features in the image, skip the current image
            if len(good_pts_current) >= min_track_features:

                # Rotation and Translation matrix calculation using Essential matrix
                Essential_Mat, e_mask = cv.findEssentialMat(good_pts_prev, good_pts_current, 
                                                            focal=focal_length, pp=(cx, cy),
                                                            method=cv.FM_RANSAC, prob=0.999, threshold=1.0, mask=cv.UMat())
                if Essential_Mat is None:

                    print('Cannot get essential matrix')
                    # Update the previous frame and previous points
                    prev_image_gray = current_image_gray.copy()
                    p0 = good_pts_current.reshape(-1, 1, 2)

                    # Create a mask image for drawing purposes
                    mask = np.zeros_like(prev_image)

                elif Essential_Mat is not None:

                    retval, Rotation_Mat, Translation_Mat, newMask = cv.recoverPose(Essential_Mat, good_pts_prev, good_pts_current, focal=focal_length, pp=(cx, cy))
                    
                    # 2D->3D Triangulation using intrinsic params, extrinsic params (Transformation Matrix), and tracking features
                    # Acquire 3-D points of tracking features in the world with triangulation
                    extrinsicCamMat = np.hstack((Rotation_Mat, Translation_Mat))

                    good_pts_current_tri = good_pts_current.reshape(2, -1)
                    good_pts_prev_tri = good_pts_prev.reshape(2, -1)

                    #current_cloud = cv.triangulatePoints(intrinsicCamMat, extrinsicCamMat, good_pts_current_tri, good_pts_prev_tri).reshape(-1, 4)[:, :3]
                    # The canonical matrix (set as the origin)
                    P0 = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0]])
                    P0 = intrinsicCamMat.dot(P0)
                    # Rotated and translated using P0 as the reference point
                    P1 = np.hstack((Rotation_Mat, Translation_Mat))
                    P1 = intrinsicCamMat.dot(P1)

                    current_cloud = cv.triangulatePoints(P0, P1, good_pts_prev_tri, good_pts_current_tri).reshape(-1, 4)[:, :3]

                    # Relative scale computation using the distance between 3-D points of tracking features
                    print('prev_cloud shape : ', prev_cloud.shape)
                    print('current_cloud shape : ', current_cloud.shape)

                    # Between two 3-D points of tracking features from t image, t-1 image, t-2 image, 
                    # select the common 3-D points
                    common_cloud_num = min([prev_cloud.shape[0], current_cloud.shape[0]])
                    print('Number of common 3-D features points : ', common_cloud_num)

                    # Process common 3-D points for relative scaling
                    ratios = []
                    for i in range(common_cloud_num):
                        if i > 0:   # Pick a pair of common 3-D points and use them to calculate the depth (disparsity)
                            
                            # A pair of common 3-D points from current clouds
                            current_Xk1 = current_cloud[i]
                            current_Xk2 = current_cloud[i-1]

                            # A pair of common 3-D points from previous clouds
                            prev_Xk1 = prev_cloud[i]
                            prev_Xk2 = prev_cloud[i-1]

                            # Use np.linalg.norm to acquire L2 norm between a pair of common 3-D points, which serves as the distance between the pair
                            if np.linalg.norm(current_Xk1 - current_Xk2 != 0):
                                ratios.append(np.linalg.norm(prev_Xk1 - prev_Xk2) / np.linalg.norm(current_Xk1 - current_Xk2))

                    T_relative_scale = np.median(ratios)    # Use median value as the final relative scale result

                    print('Relative Scale for Translation : ', T_relative_scale)
                    
                    # Consider dominant forward motion
                    # This is because when the vehicle is standing still, there should not be changes in feature coordinates.
                    # As a result, when standing still, transformation has to be zero. Therefore, consider forward translation as dominant component in camera pose.
                    if(Translation_Mat[2] > Translation_Mat[0]) and (Translation_Mat[2] > Translation_Mat[1]):
                        pose_T = pose_T + T_relative_scale * pose_R.dot(Translation_Mat)
                        pose_R = Rotation_Mat.dot(pose_R)

                    print('---Translation_Mat---')
                    print(Translation_Mat)

                    print('---Rotation_Mat---')
                    print(Rotation_Mat)
                    
                    print('---Pose_T---')
                    print(pose_T)

                    # Update the previous frame and previous points
                    prev_image_gray = current_image_gray.copy()
                    p0 = good_pts_current.reshape(-1, 1, 2)

                    prev_cloud = current_cloud
                    ref_ready = True    # With reference cloud points acquired, move onto main processing step
                    
                    # Create a mask image for drawing purposes
                    mask = np.zeros_like(prev_image)
                    
                    # Draw the trajectory #########################################################
                    plt.xlim(-100.0, 100.0)
                    plt.ylim(-100.0, 100.0)
                    plt.scatter(pose_T[0], pose_T[2])

                    plt.draw()
                    plt.show(block=False)
                    plt.pause(0.0001)

                    # Draw the tracks #############################################################
                    for i, (current, prev) in enumerate(zip(good_pts_current, good_pts_prev)):
                        a, b = current.ravel()
                        c, d = prev.ravel()
                        mask = cv.line(mask, (a, b), (c, d), color[i%100].tolist(), 2)
                        current_image = cv.circle(current_image, (a, b), 5, color[i%100].tolist(), -1)

                    img = cv.add(current_image, mask)

                    cv.imshow('Optical Flow Feature Tracking', img)

                    k = cv.waitKey(30) & 0xff
                    if k == 27:
                        break
                    ###############################################################################

            else:
                print('Not good enought features')
                prev_image_gray = current_image_gray.copy()
                p0 = cv.goodFeaturesToTrack(prev_image_gray, mask=None, **feature_params)

                # Create a mask image for drawing purposes
                mask = np.zeros_like(prev_image)

        # If optical flow fails to track features,
        # reset the previous image with current image and re-collect features for tracking
        else:
            print('Not good enought features')
            prev_image_gray = current_image_gray.copy()
            p0 = cv.goodFeaturesToTrack(prev_image_gray, mask=None, **feature_params)

            # Create a mask image for drawing purposes
            mask = np.zeros_like(prev_image)

cv.destroyAllWindows()