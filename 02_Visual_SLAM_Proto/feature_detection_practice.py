import numpy as np
import cv2 as cv
import os
from matplotlib import pyplot as plt

test_image_path = 'test_images'

images_filename = list(os.listdir(test_image_path))

detector_mode = 3

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
        cv.drawKeypoints(img, keypoints, OutImg, color=(0,255,0), flags=0)

        plt.imshow(OutImg)
        plt.show()

