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
from mpl_toolkits.mplot3d import Axes3D

import pyrealsense2 as rs

import math

print("OpenCV Ver : " + cv.__version__)

class visual_odom:
    def __init__(self):
        self.pose = [[0], [0], [0]]
        self.max_good_match_num = 2000

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        self.CameraMatrix = np.array([[rs.intrinsics().fx, 0, rs.intrinsics().ppx],
                                      [0, rs.intrinsics().fy, rs.intrinsics().ppy],
                                      [0, 0, 1]])

        self.orb = cv.ORB_create()
        self.BFMatcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

        self.pprev_frames = None
        self.pprev_image = None
        self.pprev_keypoints = None
        self.pprev_descriptors = None

        self.prev_frames = None
        self.prev_image = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_pts1 = []
        self.prev_pts2 = []
        self.prev_des1 = []
        self.prev_des2 = []
        self.prev_goodMatches = []
        self.prev_cloud = None

        self.prev_RotateM = None
        self.prev_TranslateM = None

        self.current_frames = None
        self.current_image = None
        self.current_keypoints = None
        self.current_descriptors = None
        self.current_pts1 = []
        self.current_pts2 = []
        self.current_des1 = []
        self.current_des2 = []
        self.current_goodMatches = []
        self.current_cloud = None
        
        self.current_RotateM = None
        self.current_TranslateM = None

        self.relative_scale = 0.0

        print(rs.intrinsics())
        print(rs.extrinsics())

    def processFirstFrame(self):

        frames = self.pipeline.wait_for_frames()
        self.pprev_frames = frames.get_color_frame()

        self.pprev_image = np.asanyarray(self.pprev_frames.get_data())

        self.pprev_keypoints = self.orb.detect(self.pprev_image, None)
        self.pprev_keypoints, self.pprev_descriptors = self.orb.compute(self.pprev_image, self.pprev_keypoints)

    def processSecondFrame(self):

        frames = self.pipeline.wait_for_frames()
        self.prev_frames = frames.get_color_frame()
        
        self.prev_image = np.asanyarray(self.prev_frames.get_data())

        self.prev_keypoints = self.orb.detect(self.prev_image, None)
        self.prev_keypoints, self.prev_descriptors = self.orb.compute(self.prev_image, self.prev_keypoints)

        matches = self.BFMatcher.match(self.pprev_descriptors, self.prev_descriptors)
        matches = sorted(matches, key=lambda x:x.distance)

        self.prev_goodMatches = matches[:self.max_good_match_num]

        for m in self.prev_goodMatches:
            self.prev_pts1.append(self.pprev_keypoints[m.queryIdx].pt)
            self.prev_pts2.append(self.prev_keypoints[m.trainIdx].pt)

            self.prev_des1.append(self.pprev_descriptors[m.queryIdx])
            self.prev_des2.append(self.prev_descriptors[m.trainIdx])

        self.prev_pts1 = np.float32(self.prev_pts1)
        self.prev_pts2 = np.float32(self.prev_pts2)

        Essential_Mat, mask = cv.findEssentialMat(self.prev_pts1, self.prev_pts2, focal=1.93, pp=(rs.intrinsics().ppx, rs.intrinsics().ppy), method=cv.RANSAC, prob=0.999, threshold=1.0, mask=cv.UMat())

        retval, Rotation_Mat, Translation_Mat, newMask = cv.recoverPose(Essential_Mat, self.prev_pts1, self.prev_pts2, focal=1.93, pp=(rs.intrinsics().ppx, rs.intrinsics().ppy))

        self.current_RotateM = Rotation_Mat
        self.current_TranslateM = Translation_Mat

        Transformation_Mat = np.concatenate((Rotation_Mat, Translation_Mat), axis=1)
        Transformation_Mat = np.concatenate((Transformation_Mat, [[0, 0, 0, 1]]), axis=0)

        P0 = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0]])

        P0 = self.CameraMatrix.dot(P0)

        P1 = np.hstack((Rotation_Mat, Translation_Mat))
        P1 = self.CameraMatrix.dot(P1)

        self.prev_pts1 = self.prev_pts1.reshape(2, -1)
        self.prev_pts2 = self.prev_pts2.reshape(2, -1)

        self.prev_cloud = cv.triangulatePoints(P0, P1, self.prev_pts1, self.prev_pts2).reshape(-1, 4)[:, :3]

    def processFrames(self):

        frames = self.pipeline.wait_for_frames()
        self.current_frames = frames.get_color_frame()
        
        self.current_image = np.asanyarray(self.current_frames.get_data())

        self.current_keypoints = self.orb.detect(self.current_image, None)
        self.current_keypoints, self.current_descriptors = self.orb.compute(self.current_image, self.current_keypoints)

        matches = self.BFMatcher.match(self.prev_descriptors, self.current_descriptors)
        matches = sorted(matches, key=lambda x:x.distance)

        self.current_goodMatches = matches[:self.max_good_match_num]

        for m in self.current_goodMatches:
            self.current_pts1.append(self.prev_keypoints[m.queryIdx].pt)
            self.current_pts2.append(self.current_keypoints[m.trainIdx].pt)

            self.current_des1.append(self.prev_descriptors[m.queryIdx])
            self.current_des2.append(self.current_descriptors[m.trainIdx])

        self.current_pts1 = np.float32(self.current_pts1)
        self.current_pts2 = np.float32(self.current_pts2)

        Essential_Mat, mask = cv.findEssentialMat(self.current_pts1, self.current_pts2, focal=1.93, pp=(rs.intrinsics().ppx, rs.intrinsics().ppy), method=cv.RANSAC, prob=0.999, threshold=1.0, mask=cv.UMat())

        retval, Rotation_Mat, Translation_Mat, newMask = cv.recoverPose(Essential_Mat, self.current_pts1, self.current_pts2, focal=1.93, pp=(rs.intrinsics().ppx, rs.intrinsics().ppy))

        Transformation_Mat = np.concatenate((Rotation_Mat, Translation_Mat), axis=1)
        Transformation_Mat = np.concatenate((Transformation_Mat, [[0, 0, 0, 1]]), axis=0)

        P0 = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0]])

        P0 = self.CameraMatrix.dot(P0)

        P1 = np.hstack((Rotation_Mat, Translation_Mat))
        P1 = self.CameraMatrix.dot(P1)

        self.current_pts1 = self.current_pts1.reshape(2, -1)
        self.current_pts2 = self.current_pts2.reshape(2, -1)

        self.current_cloud = cv.triangulatePoints(P0, P1, self.current_pts1, self.current_pts2).reshape(-1, 4)[:, :3]

        ### Relative scale calculation along 3 image frames using 3-D points
        min_idx = min([self.current_cloud.shape[0], self.prev_cloud.shape[0]])

        print(min_idx)

        ratios = []

        for i in range(min_idx):
            if i > 0:
                Xk = self.current_cloud[i]
                p_Xk = self.current_cloud[i-1]

                Xk_1 = self.prev_cloud[i]
                p_Xk_1 = self.prev_cloud[i-1]

                if np.linalg.norm(p_Xk - Xk) != 0:
                    ratios.append(np.linalg.norm(p_Xk_1 - Xk_1) / np.linalg.norm(p_Xk - Xk))
        
        self.relative_scale = np.median(ratios)

        print('Relative Scale : ' + str(self.relative_scale))

        ### Accept only dominant forward motion
        if ((Translation_Mat[2][0] > Translation_Mat[0][0]) and (Translation_Mat[2][0] > Translation_Mat[1][0])):
            self.current_TranslateM = self.current_TranslateM + self.relative_scale * self.current_RotateM.dot(Translation_Mat)
            self.current_RotateM = Rotation_Mat.dot(self.current_RotateM)

        print('[Updated Pose]')
        print(str(self.current_TranslateM))

        ### Update current values as previous values for next frame
        print('---Update previous values ---')
        self.prev_cloud = self.current_cloud
        self.prev_image = self.current_image
        self.prev_descriptors = self.current_descriptors
        self.prev_keypoints = self.current_keypoints

        ### Initialize appended lists as empty for next frame
        self.current_pts1 = []
        self.current_pts2 = []
        self.current_des1 = []
        self.current_des2 = []
        self.current_goodMatches = []

vOdom = visual_odom()

vOdom.processFirstFrame()
vOdom.processSecondFrame()

#map = plt.figure()
#map_ax = Axes3D(map)

#map_ax.set_xlim3d([-100.0, 100.0])
#map_ax.set_ylim3d([-100.0, 100.0])
#map_ax.set_zlim3d([-100.0, 100.0])

plt.figure()
plt.xlim(-50.0, 50.0)
plt.ylim(-50.0, 50.0)
plt.grid()

x = []
y = []
z = []

while True:
    vOdom.processFrames()
    
    print('X : ' + str(vOdom.current_TranslateM[0][0]))
    x.append(vOdom.current_TranslateM[0][0])
    
    print('Y : ' + str(vOdom.current_TranslateM[1][0]))
    y.append(vOdom.current_TranslateM[1][0]) 
    
    print('Z : ' + str(vOdom.current_TranslateM[2][0]))
    z.append(vOdom.current_TranslateM[2][0]) 
    
    #hl, = map_ax.plot3D(vOdom.prev_TranslateM[0], vOdom.prev_TranslateM[1], vOdom.prev_TranslateM[2])
    
    #hl.set_xdata(x)
    #hl.set_ydata(y)
    #hl.set_3d_properties(z)
    plt.scatter(vOdom.current_TranslateM[0][0], vOdom.current_TranslateM[1][0])
    plt.draw()
    plt.show(block=False)
    plt.pause(0.001)