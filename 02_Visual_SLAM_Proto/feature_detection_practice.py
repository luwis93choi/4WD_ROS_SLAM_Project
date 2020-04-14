import numpy as np
import cv2 as cv            # OpenCV
import os
from matplotlib import pyplot as plt

import pyrealsense2 as rs   # Realsense SDK 2.0 Python Wrapper

test_image_path = 'test_images'

images_filename = list(os.listdir(test_image_path))

detector_mode = 4

print(cv.__version__)

# Mode 0 : Harris Corner Feature Detector
# (https://docs.opencv.org/master/dc/d0d/tutorial_py_features_harris.html)
if detector_mode == 0:

    for i in range(len(images_filename)):
        print("Reading image : " + images_filename[i])

        img = cv.imread(test_image_path + '/' + images_filename[i])
        
        # Conversion into grayscale for caclulation
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        gray = np.float32(gray) # Convert grayscale image into Float32 matrix form

        # Use Harris Corner to find corners in the image
        dst = cv.cornerHarris(gray, 2, 3, 0.04)

        dst = cv.dilate(dst, None)

        img[dst > 0.01*dst.max()] = [0, 0, 255]

        cv.imshow('dst', img)

        if cv.waitKey(0) == 27:     # Press ESC to close current window
            cv.destroyAllWindows()

# Mode 1 : Refined Harris Corner Feature Detector
# (https://docs.opencv.org/master/dc/d0d/tutorial_py_features_harris.html)
elif detector_mode == 1:

    for i in range(len(images_filename)):
        print("Reading image : " + images_filename[i])

        img = cv.imread(test_image_path + '/' + images_filename[i])

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        gray = np.float32(gray)

        dst = cv.cornerHarris(gray, 2, 3, 0.04)

        dst = cv.dilate(dst, None)

        ret, dst = cv.threshold(dst, 0.01 * dst.max(), 255, 0)

        dst = np.uint8(dst)

        ret, labels, stats, centroids = cv.connectedComponentsWithStats(dst)

        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.001)
        corners = cv.cornerSubPix(gray, np.float32(centroids), (5,5), (-1,-1), criteria)

        res = np.hstack((centroids, corners))
        res = np.int0(res)
        img[res[:,1], res[:,0]] = [0, 0, 255]
        img[res[:,3], res[:,2]] = [0, 255, 0]

        cv.imshow('Refined result', img)

        if cv.waitKey(0) == 27:
            cv.destroyAllWindows()

# Mode 2 : Shi-Tomasi Corner Feature Detector
# (https://docs.opencv.org/master/d4/d8c/tutorial_py_shi_tomasi.html)
elif detector_mode == 2:

    for i in range(len(images_filename)):
        print("Reading image : " + images_filename[i])

        img = cv.imread(test_image_path + '/' + images_filename[i])

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        corners = cv.goodFeaturesToTrack(gray, 25, 0.01, 10)
        corners = np.int0(corners)

        color = 255
        index = 0

        for j in corners:

            color = 255 * (1 - (index / len(corners)))

            x, y = j.ravel()
            cv.circle(img, (x,y), 3, color, -1)
            print("Max Corner Pos" + "[" + str(index) + "] : (" + str(x) + ", " + str(y) + ")")

            index = index + 1

        plt.imshow(img)
        plt.show()

# Mode 3 : ORB (Oriented FAST and Rotated BRIEF) feature detector - Alternative algorithm for SIFT and SURF
# (https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_orb/py_orb.html)
elif detector_mode == 3:

    for i in range(len(images_filename)):
        print("Reading image : " + images_filename[i])

        img = cv.imread(test_image_path + '/' + images_filename[i])

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # SIFT (Scale-Invariant Feature Transform) and SURF (Speeded Up Robust Features) algorithms are a patented algorithm since OpenCV 4.0. It is no longer freely supported on OpenCV.
        # In order to use those algorithms, we have to build OpenCV and OpenCV-contrib from the source with OPENCV_ENABLE_NONFREE set during Cmake build.
        # https://www.pyimagesearch.com/2015/07/16/where-did-sift-and-surf-go-in-opencv-3/

        orb = cv.ORB_create()
        # ORB segmantation solution : Use ORB_create() instead of ORB()
        # (https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_orb/py_orb.html)

        keypoints = orb.detect(img, None)
        
        keypoints, des = orb.compute(img, keypoints)

        OutImg = img

        for kp in keypoints:

            print("x : " + str(kp.pt[0]) + ", y : " + str(kp.pt[1]))  

        cv.drawKeypoints(img, keypoints, OutImg, color=(0,255,0), flags=0)

        plt.imshow(OutImg)
        plt.show()

# Real-Time ORB Feature Detector using Intel Realsense D435i
elif detector_mode == 4:

    # Installation Process for ROS & Python
    #
    # 1. Install Intel Realsense SDK
    #    (1) Register the server's public key
    #        sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    #    (2) Add the server to the list of repositories
    #        - For Ubuntu 16 LTS
    #          sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
    #        - For Ubuntu 18 LTS
    #          sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
    #    (3) Install libraries
    #        - sudo apt-get install librealsense2-dkms  (Depth camera-specific kernel extensions)
    #        - sudo apt-get install librealsense2-utils (Demos and tools for Realsens SDK)
    #        - sudo apt-get install librealsense2-dev   (Header files and symbolic links for developers)
    #        - sudo apt-get install librealsense2-dbg   (Debug symbols for developers)
    #
    #          * Optional : sudo apt-get install librealsense2-udev (Configurations for Realsense Device permissions on kernel level)
    #                     : sudo apt-get install librealsense2      (Realsense SDK runtime and configuration files)
    #
    #    (4) Check & Upgrade Installation
    #        - Check Installation : modinfo uvcvideo | grep "version:" --> Check whether it includes 'realsense'
    #        - Upgrade package : sudo apt get update --> sudo apt get upgrade
    #
    # 2. Intel Realsense ROS Wrapper Package Installation ('realsense2-camera')
    #    (1) Install ROS package according to distribution version
    #        sudo apt-get install ros-(distro)-realsense2-camera
    #        (ex : For Melodic - sudo apt-get install ros-melodic-realsense2-camera)
    #
    # 3. Intel Realsense Python Wrapper Installation
    #    (1) Use pip3 to install pyrealsense2 for Python3
    #        pip3 install pyrealsense2
    #
    #    *** pyrealsense2 is official Python wrapper for Realsense SDK 2.0
    #    *** pyrealsense is Python wrapper for legacy librealsense v1.12.1 / pyrealsense does not support Realsense SDK 2.0

    pipeline = rs.pipeline()
    
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)

    try:

        orb = cv.ORB_create()

        while True:

            frames = pipeline.wait_for_frames()
            color_frames = frames.get_color_frame()

            if not color_frames:
                continue

            color_image = np.asanyarray(color_frames.get_data())

            keypoints = orb.detect(color_image, None)

            keypoints, des = orb.compute(color_image, keypoints)

            OutputImg = color_image

            cv.drawKeypoints(color_image, keypoints, OutputImg, color=(0, 255, 0), flags=0)

            cv.namedWindow('Realsense', cv.WINDOW_AUTOSIZE)
            cv.imshow('Realsense', OutputImg)
            cv.waitKey(1)

    finally:
        pipeline.stop()

