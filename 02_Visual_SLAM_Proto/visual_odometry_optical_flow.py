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

print(rs.intrinsics())
print(rs.extrinsics())

# Create some random colors
color = np.random.randint(0, 255, (100, 3))

# Take the first frame and find corners in it
frames = pipeline.wait_for_frames()
prev_frames = frames.get_color_frame()

prev_image = np.asanyarray(prev_frames.get_data())

prev_image_gray = cv.cvtColor(prev_image, cv.COLOR_BGR2GRAY)

p0 = cv.goodFeaturesToTrack(prev_image_gray, mask=None, **feature_params)

# Create a mask image for drawing purposes
mask = np.zeros_like(prev_image)

while True:

    frames = pipeline.wait_for_frames()
    current_frames = frames.get_color_frame()

    current_image = np.asanyarray(current_frames.get_data())

    current_image_gray = cv.cvtColor(current_image, cv.COLOR_RGB2GRAY)

    # Calculate Optical Flow
    p1, st, err = cv.calcOpticalFlowPyrLK(prev_image_gray, current_image_gray, p0, None, **lk_params)

    # If optical flow is successful with feature tracking,
    # collect good feature points and draw the tracks on the image
    if (p1 is not None) and (p0 is not None):

        if len(p1) >= 3:
            # Select good points
            good_pts_current = p1[st==1]
            good_pts_prev = p0[st==1]

            # Draw the tracks
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

            # Update the previous frame and previous points
            prev_image_gray = current_image_gray.copy()
            p0 = good_pts_current.reshape(-1, 1, 2)

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