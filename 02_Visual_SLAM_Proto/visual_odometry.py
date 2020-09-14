# Reference 01 : Epipolar Geometry (https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_epipolar_geometry/py_epipolar_geometry.html)
# Reference 02 : Monocular Visual Odometry - OpenCv Python (Relative Scale & Absolute Scale) (https://github.com/caomw/visual_odometry-1/blob/master/src/py_MVO.py)
# Reference 03 : SFM with OpenCV + GTSAM + PMVS / Relative Scale explanation (http://nghiaho.com/?p=2379)
# Reference 04 : Monocular Visual and Inertial Odometry (https://dattadebrup.github.io/monocular/inertial/odometry/2018/07/23/Monocular-Visual-and-Inertial-Odometry.html)
# Reference 05 : Monocular Visual Odometry (Absolute Scale only) (https://avisingh599.github.io/vision/monocular-vo/)
# Reference 06 : Tutorial on Visual Odometry (http://rpg.ifi.uzh.ch/docs/Visual_Odometry_Tutorial.pdf)
# Reference 07 : Visual Odometry (http://www.cs.toronto.edu/~urtasun/courses/CSC2541/03_odometry.pdf)
# Reference 08 : Sturcture from Motion - Triangulation (http://www.cs.cmu.edu/~16385/s17/Slides/11.4_Triangulation.pdf)
# Reference 09 : Relative Scale in SfM (https://robotics.stackexchange.com/questions/14471/relative-scale-in-sfm)
# Reference 10 : Rotation Matrix, Translation Matrix, Transformatio Matrix (https://answers.opencv.org/question/96474/projectpoints-functionality-question/)
# Reference 11 : OpenCV Triangulation API (https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)

import numpy as np
import cv2 as cv
import os
import sys

from matplotlib import pyplot as plt

import pyrealsense2 as rs

import math

print("OpenCV Ver : " + cv.__version__)

pose = []
max_good_match_num = 20

pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

device = pipeline.start(config).get_device()

print(rs.intrinsics())
print(rs.extrinsics())

try:

    orb = cv.ORB_create()

    brute_force_matcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

    ### Initial Feature Extraction ###
    
    # Need to store at least 3 frames/images for relative scale calculation
    # Prepare 1st and 2nd frame
    frames = pipeline.wait_for_frames()
    current_frames = frames.get_color_frame()

    current_image = np.asanyarray(current_frames.get_data())

    current_keypoints = orb.detect(current_image, None)
    current_keypoints, current_descriptors = orb.compute(current_image, current_keypoints)

    pprev_image = current_image
    pprev_keypoints = current_keypoints
    pprev_descriptors = current_descriptors

    # Prepare 3rd frame and update
    frames = pipeline.wait_for_frames()
    current_frames = frames.get_color_frame()

    current_image = np.asanyarray(current_frames.get_data())

    current_keypoints = orb.detect(current_image, None)
    current_keypoints, current_descriptors = orb.compute(current_image, current_keypoints)

    prev_image = current_image
    prev_keypoints = current_keypoints
    prev_descriptors = current_descriptors

    # Prepare previous matches (k-1 matches) for relative scale calculation
    prev_matches = brute_force_matcher.match(pprev_descriptors, prev_descriptors)
    prev_matches = sorted(prev_matches, key=lambda x:x.distance)
    prev_good_matches = prev_matches[:max_good_match_num]

    ### Feature Extraction, Matching, Essential Matrix Caculation ###
    pose = [[0], [0], [0], [1]]
    is_first_time = True
    while True:
    
        frames = pipeline.wait_for_frames()
        current_frames = frames.get_color_frame()

        current_image = np.asanyarray(current_frames.get_data())

        current_keypoints = orb.detect(current_image, None)
        current_keypoints, current_descriptors = orb.compute(current_image, current_keypoints)

        matches = brute_force_matcher.match(prev_descriptors, current_descriptors)

        matches = sorted(matches, key=lambda x:x.distance)

        ### Compute Essential matrix in order to acquire rotation matrix and translation matrix between previous image axes and current image axes        
        
        good_matches = matches[:max_good_match_num]     # Use Top 10 BF matches for essential matrix calculation
        pts1 = []
        pts2 = []

        des1 = []
        des2 = []

        # Compile the keypoints from Top 10 BF matches
        for m in good_matches:
            pts1.append(prev_keypoints[m.queryIdx].pt)
            pts2.append(current_keypoints[m.trainIdx].pt)

            des1.append(prev_descriptors[m.queryIdx])
            des2.append(current_descriptors[m.trainIdx])

        pts1 = np.float32(pts1)
        pts2 = np.float32(pts2)

        # Compute Eseential Matrix from the keypoints of Top 10 BF matches, camera intrinsic parameters, and camera extrinsic parameters
        # Use RANSAC when calculating essential matrix
        Essential_Mat, mask = cv.findEssentialMat(pts1, pts2, focal=1.93, pp=(rs.intrinsics().ppx, rs.intrinsics().ppy), method=cv.RANSAC, prob=0.999, threshold=1.0, mask=cv.UMat())

        retval, Rotation_Mat, Translation_Mat, newMask = cv.recoverPose(Essential_Mat, pts1, pts2, focal=1.93, pp=(rs.intrinsics().ppx, rs.intrinsics().ppy))

        transformation_Mat = np.concatenate((Rotation_Mat, Translation_Mat), axis=1)
        transformation_Mat = np.concatenate((transformation_Mat, [[0, 0, 0, 1]]), axis=0)

        ###################################################################
        ### Explicit triangulation of 3-D points in order to acquire 'scale ratio' for translation 
        ### Monocular visual odometry suffers from scale uncertainty. It needs scale ratio for translation.
        ### Use features matches over 3 images in order to 'triangulate' relative scale

        # Compile the prev_keypoints from previous Top 10 BF matches
        prev_pts1 = []
        prev_pts2 = []
        
        prev_des1 = []
        prev_des2 = []

        for p_m in prev_good_matches:
            prev_pts1.append(pprev_keypoints[p_m.queryIdx].pt)
            prev_pts2.append(prev_keypoints[p_m.trainIdx].pt)

            prev_des1.append(pprev_descriptors[p_m.queryIdx])
            prev_des2.append(prev_descriptors[p_m.trainIdx])

        prev_pts1 = np.float32(prev_pts1)
        prev_pts2 = np.float32(prev_pts2)
        
        ###################################################################
        # Between Frame i-2 and i-1

        # Between Frame i-1 and i
        K = np.array([[rs.intrinsics().fx, 0, rs.intrinsics().ppx],
                      [0, rs.intrinsics().fy, rs.intrinsics().ppy],
                      [0, 0, 1]])

        P0 = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0]])

        P0 = K.dot(P0)

        P1 = np.hstack((Rotation_Mat, Translation_Mat))
        P1 = K.dot(P1)
        
        pts1 = pts1.reshape(2, -1)
        pts2 = pts2.reshape(2, -1)
        
        cloud_new = cv.triangulatePoints(P0, P1, pts1, pts2).reshape(-1, 4)[:, :3]
        #threeD_coords = cv.convertPointsFromHomogeneous(cloud_new.transpose())
        
        ###################################################################

        print('---------------------------------------------')
        print(Rotation_Mat)
        print(Translation_Mat)
        print('---------------------------------------------')
        #print(transformation_Mat)
        print(cloud_new)
        print('---------------------------------------------')

        pose = np.dot(transformation_Mat, pose)

        print(pose)

        match_result_img = cv.drawMatches(prev_image, prev_keypoints, current_image, current_keypoints, good_matches, None, flags=2)

        cv.namedWindow('Real-time Brute Force Matching Result', cv.WINDOW_AUTOSIZE)
        cv.imshow('Real-time Brute Force Matching Result', match_result_img)
        cv.waitKey(0)   # Press Enter to continue for next image frames


        pprev_image = prev_image
        pprev_keypoints = prev_keypoints
        pprev_descriptors = prev_descriptors

        prev_good_matches = good_matches    # Save top feature matches for next relative scale calculation
        prev_image = current_image
        prev_keypoints = current_keypoints
        prev_descriptors = current_descriptors

finally:
    pipeline.stop()