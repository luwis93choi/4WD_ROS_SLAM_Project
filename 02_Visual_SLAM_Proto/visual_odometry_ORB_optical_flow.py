import cv2 as cv
import numpy as np
import os
import sys

import pyrealsense2 as rs

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from time import sleep

print('OpenCV Ver : ', cv.__version__)

image_buffer = []   # image_buffer : [pprev_image, prev_image, current_image]
                    # ==> Store 3 consecutive images for translation rescaling

# Intel Realsense image pipeline initialization
realsense_pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = realsense_pipeline.start(config)

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

# ORB Feature Extractor
orb = cv.ORB_create()

# Brute Force Matcher
BF_Matcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

# Params for Lucas-Kanade Optical Flow
lk_params = dict(winSize = (21, 21),
                 maxLevel = 3,
                 criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 30, 0.01))

def load_realsense_frames(image_buffer, image_pipeline):

    # Initialization : Load first 3 image frames into the buffer
    if len(image_buffer) < 3:

        # Load 1st image frame into the image buffer
        frames = image_pipeline.wait_for_frames()
        frame_pprev = frames.get_color_frame()
        image_pprev = np.asanyarray(frame_pprev.get_data())
        image_buffer.append(image_pprev)

        print('[INFO] Loaded 1st image frame into the image buffer')
        
        # Load 2nd image frame into the image buffer
        frames = image_pipeline.wait_for_frames()
        frame_prev = frames.get_color_frame()
        image_prev = np.asanyarray(frame_prev.get_data())
        image_buffer.append(image_prev)

        print('[INFO] Loaded 2nd image frame into the image buffer')
    
        # Load 3rd image frame into the image buffer
        frames = image_pipeline.wait_for_frames()
        frame_current = frames.get_color_frame()
        image_current = np.asanyarray(frame_current.get_data())
        image_buffer.append(image_current)

        print('[INFO] Loaded 3rd image frame into the image buffer')
       
    # While visual odometry process is running, update the image buffer with the latest image frame
    else:

        # Push out the oldest image frame from the image buffer
        image_buffer[0] = image_buffer[1]
        image_buffer[1] = image_buffer[2]

        # Push the latest image frame into the image buffer
        frames = image_pipeline.wait_for_frames()
        frame_current = frames.get_color_frame()
        image_current = np.asanyarray(frame_current.get_data())
        image_buffer[2] = image_current

        print('[INFO] Updated the image buffer with the latest image frame')

    return image_buffer

def processFirstFrame(pprev_image, feature_extractor):

    pprev_keypoints = feature_extractor.detect(pprev_image, None)
    pprev_keypoints, pprev_descriptors = feature_extractor.compute(pprev_image, pprev_keypoints)

    print('[INFO] pprev Image ORB Extraction Results - Feature Num : ', len(pprev_keypoints))

    return pprev_keypoints, pprev_descriptors

def processSecondFrame(pprev_image, prev_image, pprev_keypoints, pprev_descriptors, feature_extractor, feature_matcher):

    prev_keypoints = feature_extractor.detect(prev_image)
    prev_keypoints, prev_descriptors = feature_extractor.compute(prev_image, prev_keypoints)

    print('[INFO] prev Image ORB Extraction Results - Feature Num : ', len(prev_keypoints))

    prev_feature_matches = feature_matcher.match(pprev_descriptors, prev_descriptors)
    prev_feature_matches = sorted(prev_feature_matches, key=lambda x:x.distance)

    print('[INFO] pprev-prev ORB Feature Match Num : ', len(prev_feature_matches))

    # Compile the keypoints and descriptors from feature matching results
    pprev_track_pts = []
    prev_track_pts = []

    pprev_track_des = []
    prev_track_des = []

    for m in prev_feature_matches:
        pprev_track_pts.append(pprev_keypoints[m.queryIdx].pt)
        prev_track_pts.append(prev_keypoints[m.trainIdx].pt)

        pprev_track_des.append(pprev_descriptors[m.queryIdx])
        prev_track_des.append(prev_descriptors[m.trainIdx])

    pprev_track_pts = np.int32(pprev_track_pts)
    prev_track_pts = np.int32(prev_track_pts)

    # Compute Essential Matrix between pprev image and prev image 
    # in order to acquire the translation and rotation between t-2 and t-1 images
    Essential_Mat, e_mask = cv.findEssentialMat(pprev_track_pts, prev_track_pts, 
                                                focal=focal_length, pp=(cx, cy),
                                                method=cv.FM_RANSAC, prob=0.999, threshold=1.0, mask=cv.UMat())

    print('[INFO] pprev-prev Essential Matrix')
    print(Essential_Mat)

    retval, Rotation_Mat, Translation_Mat, newMask = cv.recoverPose(Essential_Mat, pprev_track_pts, prev_track_pts, 
                                                                    focal=focal_length, pp=(cx, cy))

    print('[INFO] pprev-prev Translation Matrix (Unscaled)')
    print(Translation_Mat)

    print('[INFO] pprev-prev Rotation Matrix')
    print(Rotation_Mat)
    
    return pprev_track_pts, pprev_track_des, prev_track_pts, prev_track_des

'''
def processThirdFrame(prev_image, current_image):

def pose_estimate(Translate_Mat, Rotate_Mat, prev_3D_cloud, current_3D_cloud):
'''

while True:

    load_realsense_frames(image_buffer, realsense_pipeline)

    pprev_keypoints, pprev_descriptors = processFirstFrame(image_buffer[0], orb)

    processSecondFrame(image_buffer[0], image_buffer[1], pprev_keypoints, pprev_descriptors, orb, BF_Matcher)

    full_view = np.hstack((image_buffer[0], image_buffer[1], image_buffer[2]))

    cv.imshow('Images in the buffer [pprev, prev, current]', full_view)

    k = cv.waitKey(30) & 0xff
    if k == 27:
        break

    print('-----------------------------------------------------')