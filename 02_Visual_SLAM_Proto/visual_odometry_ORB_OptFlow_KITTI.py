import cv2 as cv
import numpy as np
import os
import sys

import pyrealsense2 as rs

import g2o
from g2o_BA_class import BundleAdjustment
from scipy.spatial.transform import Rotation as R

from matplotlib import pyplot as plt
from matplotlib import gridspec
from mpl_toolkits.mplot3d import Axes3D

from time import sleep

import csv

import math

class mono_VO_ORBFlow_KITTI:

    def __init__(self, _focal_length, _fx, _fy, _cx, _cy, _dataset_path, _dataset_pose_path):

        self.image_buffer = [None, None, None]   # image_buffer : [pprev_image, prev_image, current_image]
                                                 # ==> Store 3 consecutive images for translation rescaling

        self.img_features_buffer = [None, None, None]
        self.common_img_features = []
        self.common_3D_clouds = []
        
        # Camera intrinsic matrix params for 2D-3D triangulation
        self.focal_length = _focal_length
        self.fx = _fx
        self.fy = _fy
        self.cx = _cx
        self.cy = _cy

        self.intrinsic_CAM_Mat = np.array([[self.fx, 0,       self.cx],
                                           [0,       self.fy, self.cy],
                                           [0,       0,       1      ]])

        print('focal length : ', self.focal_length)
        print('fx : ', self.fx)
        print('fy : ', self.fy)
        print('ppx : ', self.cx)
        print('ppy : ', self.cy)

        self.processing_Stage = '1st'

        # ORB Feature Extractor
        self.orb = cv.ORB_create()

        # Optical Flow Parameters
        self.lk_params = dict( winSize  = (15,15),
                               maxLevel = 2,
                               criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

        self.init_optical = True
        self.initial_common_match_num = 0

        self.min_matches_num = 8

        self.pose_T = np.array([[0],
                                [0],
                                [0]])

        self.pose_R = np.array([[1, 0, 0],
                                [0, 1, 0],
                                [0, 0, 1]])
                
        self.prev_pose_T = np.array([[0],
                                     [0],
                                     [0]])

        self.prev_pose_R = np.array([[1, 0, 0],
                                     [0, 1, 0],
                                     [0, 0, 1]])
                
        self.pprev_pose_T = np.array([[0],
                                      [0],
                                      [0]])

        self.pprev_pose_R = np.array([[1, 0, 0],
                                      [0, 1, 0],
                                      [0, 0, 1]])

        self.pprev_prev_cloud = None
        self.prev_current_cloud = None

        self.dataset_path = _dataset_path
        self.images = sorted(os.listdir(_dataset_path))
        self.dataset_end_idx = len(os.listdir(_dataset_path)) - 1
        self.dataset_current_idx = 0

        # Prepare groundtruth list
        self.ground_truth_T = []
        f = open(_dataset_pose_path, 'r')
        init_flag = 0
        while True:
            line = f.readline()
            if not line: break

            pose = line.strip().split()

            self.ground_truth_T.append([float(pose[3]), float(pose[7]), float(pose[11])])
            
            # Set up initial pprev_pose as 0th groundtruth
            if init_flag == 0:
                init_flag = 1
                self.pprev_pose_T = np.array([[float(pose[3])],
                                              [float(pose[7])],
                                              [float(pose[11])]])

                self.pprev_pose_R = np.array([[float(pose[0]), float(pose[1]), float(pose[2])],
                                              [float(pose[4]), float(pose[5]), float(pose[6])],
                                              [float(pose[8]), float(pose[9]), float(pose[10])]])
            
            # Set up inital prev_pose as 1st ground truth
            elif init_flag == 1:
                init_flag = 2
                self.prev_pose_T = np.array([[float(pose[3])],
                                             [float(pose[7])],
                                             [float(pose[11])]])

                self.prev_pose_R = np.array([[float(pose[0]), float(pose[1]), float(pose[2])],
                                             [float(pose[4]), float(pose[5]), float(pose[6])],
                                             [float(pose[8]), float(pose[9]), float(pose[10])]])
            
        f.close()
        
        print('Dataset Path : ', self.dataset_path)
        print('Ground Truth Path : ', _dataset_pose_path)

    def img_buffer_feature_tracking(self, disp_img=True):

        # At 1st Frame image, conduct ORB feature extraction and convert it into an appropriate numpy format for optical flow
        if self.processing_Stage == '1st':
            print('[INFO] 1st image')

            query_img = cv.imread(self.dataset_path + self.images[self.dataset_current_idx])
            
            pprev_gray = cv.cvtColor(query_img, cv.COLOR_BGR2GRAY)
            self.image_buffer[0] = pprev_gray

            # Grayscale ORB feature extraction
            pprev_keypoints = self.orb.detect(pprev_gray, None)
            pprev_keypoints, pprev_descriptors = self.orb.compute(pprev_gray, pprev_keypoints)
            pprev_keypoints = np.array((pprev_keypoints))

            # Datatype converison for optical flow
            temp_pprev_keypoints = []
            for keypoint in pprev_keypoints:
                temp_pprev_keypoints.append([[np.float32(keypoint.pt[0]), np.float32(keypoint.pt[1])]])
            
            self.img_features_buffer[0] = np.array(temp_pprev_keypoints)

            self.processing_Stage = '2nd'
            self.dataset_current_idx += 1
        
        elif self.processing_Stage == '2nd':
            print('[INFO] 2nd image')

            query_img = cv.imread(self.dataset_path + self.images[self.dataset_current_idx])
            
            prev_gray = cv.cvtColor(query_img, cv.COLOR_BGR2GRAY)
            self.image_buffer[1] = prev_gray

            # Optical Flow Feature Tracking
            self.img_features_buffer[1], st, err = cv.calcOpticalFlowPyrLK(self.image_buffer[0], self.image_buffer[1], 
                                                                           self.img_features_buffer[0], None, **self.lk_params)

            self.img_features_buffer[0] = (self.img_features_buffer[0][st==1]).reshape(-1, 1, 2)
            self.img_features_buffer[1] = (self.img_features_buffer[1][st==1]).reshape(-1, 1, 2)

            print('[INFO] pprev-prev common feature num : ', len(self.img_features_buffer[1]))

            ### Feature Retracking over 2 images ###########################################################################################################
            if len(self.img_features_buffer[0]) < 10:
                print('[Re-Tracking] Not Enough Features between pprev and prev')

                # pprev Grayscale ORB feature extraction
                pprev_keypoints = self.orb.detect(self.image_buffer[0], None)
                pprev_keypoints, pprev_descriptors = self.orb.compute(self.image_buffer[0], pprev_keypoints)
                pprev_keypoints = np.array((pprev_keypoints))

                # pprev Datatype converison for optical flow
                temp_pprev_keypoints = []
                for keypoint in pprev_keypoints:
                    temp_pprev_keypoints.append([[np.float32(keypoint.pt[0]), np.float32(keypoint.pt[1])]])

                self.img_features_buffer[0] = np.array(temp_pprev_keypoints)

                # Re-Track between pprev and prev
                self.img_features_buffer[1], st, err = cv.calcOpticalFlowPyrLK(self.image_buffer[0], self.image_buffer[1], 
                                                                               self.img_features_buffer[0], None, **self.lk_params)

                self.img_features_buffer[0] = (self.img_features_buffer[0][st==1]).reshape(-1, 1, 2)
                self.img_features_buffer[1] = (self.img_features_buffer[1][st==1]).reshape(-1, 1, 2)
            ###################################################################################################################################################

            self.processing_Stage = 'process'
            self.dataset_current_idx += 1

        elif self.processing_Stage == 'process':

            query_img = cv.imread(self.dataset_path + self.images[self.dataset_current_idx])

            current_gray = cv.cvtColor(query_img, cv.COLOR_BGR2GRAY)
            self.image_buffer[2] = current_gray

            # Optical Flow Feature Tracking
            self.img_features_buffer[2], st, err = cv.calcOpticalFlowPyrLK(self.image_buffer[1], self.image_buffer[2], 
                                                                           self.img_features_buffer[1], None, **self.lk_params)

            self.img_features_buffer[1] = (self.img_features_buffer[1][st==1]).reshape(-1, 1, 2)
            self.img_features_buffer[2] = (self.img_features_buffer[2][st==1]).reshape(-1, 1, 2)

            self.img_features_buffer[0] = (self.img_features_buffer[0][st==1]).reshape(-1, 1, 2)

            if self.init_optical == True:
                self.initial_common_match_num = len(self.img_features_buffer[2])
                self.init_optical = False

            print('[INFO] pprev-prev-current common feature num : ', len(self.img_features_buffer[2]))

            print('pprev common num : ', len(self.img_features_buffer[0]))
            print('prev common num : ', len(self.img_features_buffer[1]))
            print('current common num : ', len(self.img_features_buffer[2]))
            print('--------------------------------')

            ### Feature Retracking over all 3 images ###
            if len(self.img_features_buffer[2]) < (self.initial_common_match_num * 0.25):
                print('[Re-Tracking] Not Enough Features between pprev and prev / Too many have been consumed')

                ### pprev-prev ###
                # pprev Grayscale ORB feature extraction
                pprev_keypoints = self.orb.detect(self.image_buffer[0], None)
                pprev_keypoints, pprev_descriptors = self.orb.compute(self.image_buffer[0], pprev_keypoints)
                pprev_keypoints = np.array((pprev_keypoints))

                # pprev Datatype converison for optical flow
                temp_pprev_keypoints = []
                for keypoint in pprev_keypoints:
                    temp_pprev_keypoints.append([[np.float32(keypoint.pt[0]), np.float32(keypoint.pt[1])]])

                self.img_features_buffer[0] = np.array(temp_pprev_keypoints)

                # Re-Track between pprev and prev
                self.img_features_buffer[1], st, err = cv.calcOpticalFlowPyrLK(self.image_buffer[0], self.image_buffer[1], 
                                                                               self.img_features_buffer[0], None, **self.lk_params)

                self.img_features_buffer[0] = (self.img_features_buffer[0][st==1]).reshape(-1, 1, 2)
                self.img_features_buffer[1] = (self.img_features_buffer[1][st==1]).reshape(-1, 1, 2)

                ### prev-current ###
                # pprev Grayscale ORB feature extraction
                prev_keypoints = self.orb.detect(self.image_buffer[1], None)
                prev_keypoints, pprev_descriptors = self.orb.compute(self.image_buffer[1], prev_keypoints)
                prev_keypoints = np.array((prev_keypoints))

                # pprev Datatype converison for optical flow
                temp_prev_keypoints = []
                for keypoint in prev_keypoints:
                    temp_prev_keypoints.append([[np.float32(keypoint.pt[0]), np.float32(keypoint.pt[1])]])

                self.img_features_buffer[1] = np.array(temp_prev_keypoints)

                # Re-Track between pprev and prev
                self.img_features_buffer[2], st, err = cv.calcOpticalFlowPyrLK(self.image_buffer[1], self.image_buffer[2], 
                                                                               self.img_features_buffer[1], None, **self.lk_params)

                self.img_features_buffer[1] = (self.img_features_buffer[1][st==1]).reshape(-1, 1, 2)
                self.img_features_buffer[2] = (self.img_features_buffer[2][st==1]).reshape(-1, 1, 2)

                # Update common cloud number
                self.initial_common_match_num = len(self.img_features_buffer[2])

            ############################################

            # Display Optical Flow results if the display option is True
            if disp_img == True:

                color = np.random.randint(0, 255, (2000, 3)) 

                query_img = cv.imread(self.dataset_path + self.images[self.dataset_current_idx-1])
                
                # Display Optical Flow Results
                mask = np.zeros_like(query_img)

                for i, (new, old) in enumerate(zip(self.img_features_buffer[1], self.img_features_buffer[2])):
                    a, b = new.ravel()
                    c, d = old.ravel()
                    mask = cv.line(mask, (a, b), (c, d), color[i].tolist(), 2)
                    query_img = cv.circle(query_img, (a, b), 5, color[i].tolist(), -1)

                img = cv.add(query_img.copy(), mask)

                cv.imshow('ORB-based Optical Flow + ReTracking', img)
                k = cv.waitKey(30)

            # Update current values as prev values
            self.image_buffer[0] = self.image_buffer[1]
            self.img_features_buffer[0] = self.img_features_buffer[1]

            self.image_buffer[1] = self.image_buffer[2]
            self.img_features_buffer[1] = self.img_features_buffer[2]

            self.dataset_current_idx += 1

