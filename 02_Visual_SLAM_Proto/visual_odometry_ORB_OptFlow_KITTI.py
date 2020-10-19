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
        self.lk_params = dict( winSize  = (15, 15),
                               maxLevel = 2,
                               criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

        self.init_optical = True
        self.init_cloud = True
        self.initial_common_match_num = 0
        self.retracking_ratio = 0.5

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

        self.geometric_unit_changes = {'R_pprev_prev' : None, 'T_pprev_prev' : None, 
                                       'R_prev_current' : None, 'T_prev_current' : None}

        ### Prepare groundtruth list #####################################################################
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
        #####################################################################################################

        print('Dataset Path : ', self.dataset_path)
        print('Ground Truth Path : ', _dataset_pose_path)

    def frame_Skip(self):

        pprev_match_keypoints_pts = np.float32(self.img_features_buffer[0])
        prev_match_keypoints_pts = np.float32(self.img_features_buffer[1])
        current_match_keypoints_pts = np.float32(self.img_features_buffer[2])

        pixel_diff = np.mean(abs(current_match_keypoints_pts - prev_match_keypoints_pts))

        print('[Pixel DIFF] : ',  pixel_diff)

        return pixel_diff < 3

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

            ### Feature Retracking over all 3 images ###################################################################################
            if len(self.img_features_buffer[2]) < (self.initial_common_match_num * self.retracking_ratio):
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

                # Re-Track between prev and current
                self.img_features_buffer[2], st, err = cv.calcOpticalFlowPyrLK(self.image_buffer[1], self.image_buffer[2], 
                                                                               self.img_features_buffer[1], None, **self.lk_params)

                self.img_features_buffer[1] = (self.img_features_buffer[1][st==1]).reshape(-1, 1, 2)
                self.img_features_buffer[2] = (self.img_features_buffer[2][st==1]).reshape(-1, 1, 2)

                self.img_features_buffer[0] = (self.img_features_buffer[0][st==1]).reshape(-1, 1, 2)

                # Update common cloud number
                self.initial_common_match_num = len(self.img_features_buffer[2])
            
                self.init_cloud = True

            ##################################################################################################################################

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

            self.dataset_current_idx += 1

    def geometric_change_calc(self):
        
        pprev_match_keypoints_pts = self.img_features_buffer[0]

        prev_match_keypoints_pts = self.img_features_buffer[1]

        current_match_keypoints_pts = self.img_features_buffer[2]

        ### Essential Matrix Calcuation & Rotation/Translation Matrix Calculation ###
        Essential_Mat_pprev_prev, mask_pprev_prev = cv.findEssentialMat(np.int32(prev_match_keypoints_pts), 
                                                                        np.int32(pprev_match_keypoints_pts),
                                                                        cameraMatrix=self.intrinsic_CAM_Mat,
                                                                        method=cv.RANSAC, prob=0.999, threshold=1.0)

        Essential_Mat_prev_current, mask_prev_current = cv.findEssentialMat(np.int32(current_match_keypoints_pts), 
                                                                            np.int32(prev_match_keypoints_pts),
                                                                            cameraMatrix=self.intrinsic_CAM_Mat,
                                                                            method=cv.RANSAC, prob=0.999, threshold=1.0)

        retval, Rotation_Mat_pprev_prev, Translation_Mat_pprev_prev, r_mask_pprev_prev = cv.recoverPose(Essential_Mat_pprev_prev,
                                                                                                        np.int32(prev_match_keypoints_pts),
                                                                                                        np.int32(pprev_match_keypoints_pts),
                                                                                                        cameraMatrix=self.intrinsic_CAM_Mat)

        retval, Rotation_Mat_prev_current, Translation_Mat_prev_current, r_mask_prev_current = cv.recoverPose(Essential_Mat_prev_current,
                                                                                                              np.int32(current_match_keypoints_pts),
                                                                                                              np.int32(prev_match_keypoints_pts),
                                                                                                              cameraMatrix=self.intrinsic_CAM_Mat)

        self.geometric_unit_changes['R_pprev_prev'] = Rotation_Mat_pprev_prev
        self.geometric_unit_changes['T_pprev_prev'] = Translation_Mat_pprev_prev
        self.geometric_unit_changes['R_prev_current'] = Rotation_Mat_prev_current
        self.geometric_unit_changes['T_prev_current'] = Translation_Mat_prev_current

        print('[R_pprev_prev]')
        print(Rotation_Mat_pprev_prev)
        print('[T_pprev_prev]')
        print(Translation_Mat_pprev_prev)
        print('[R_prev_current]')
        print(Rotation_Mat_prev_current)
        print('[T_prev_current]')
        print(Translation_Mat_prev_current)

        return self.geometric_unit_changes

    def img_common3Dcloud_triangulate(self):
        
        pprev_match_keypoints_pts = np.float32(self.img_features_buffer[0]).reshape(2, -1)
        prev_match_keypoints_pts = np.float32(self.img_features_buffer[1]).reshape(2, -1)
        current_match_keypoints_pts = np.float32(self.img_features_buffer[2]).reshape(2, -1)

        Rotation_Mat_pprev_prev = self.geometric_unit_changes['R_pprev_prev']
        Translation_Mat_pprev_prev = self.geometric_unit_changes['T_pprev_prev']
        Rotation_Mat_prev_current = self.geometric_unit_changes['R_prev_current']
        Translation_Mat_prev_current = self.geometric_unit_changes['T_prev_current']

        if self.init_cloud == True:
            print('[INFO] pprev-prev Triangulation')
            ### Triangluation between pprev and prev
            # The canonical matrix (set as the origin)
            P0 = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0]])
            P0 = self.intrinsic_CAM_Mat.dot(P0)
            # Rotated and translated using P0 as the reference point
            P1 = np.hstack((Rotation_Mat_pprev_prev, Translation_Mat_pprev_prev))
            P1 = self.intrinsic_CAM_Mat.dot(P1)

            self.pprev_prev_cloud = cv.triangulatePoints(P0, P1, pprev_match_keypoints_pts, prev_match_keypoints_pts).reshape(-1, 4)[:, :3]

            self.init_cloud = False

        print('[INFO] prev-current Triangulation')
        ### Triangluation between prev and current
        # The canonical matrix (set as the origin)
        P0 = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0]])
        P0 = self.intrinsic_CAM_Mat.dot(P0)
        # Rotated and translated using P0 as the reference point
        P1 = np.hstack((Rotation_Mat_prev_current, Translation_Mat_prev_current))
        P1 = self.intrinsic_CAM_Mat.dot(P1)

        self.prev_current_cloud = cv.triangulatePoints(P0, P1, prev_match_keypoints_pts, current_match_keypoints_pts).reshape(-1, 4)[:, :3]

        
        # Remove the triangulation results with negative Z value 
        # (Remove the triangulation values that are projected behind image plane)
        #corrected_pprev_prev_cloud = []
        #corrected_prev_current_cloud = []
        #for i in range(len(pprev_prev_cloud)):
        #    if (pprev_prev_cloud[i][2] >= 0) and (prev_current_cloud[i][2] >= 0):
        #        corrected_pprev_prev_cloud.append(pprev_prev_cloud[i])
        #        corrected_prev_current_cloud.append(prev_current_cloud[i])
        #
        #return corrected_pprev_prev_cloud, corrected_prev_current_cloud
        
        return self.pprev_prev_cloud, self.prev_current_cloud

    def pose_estimate(self):

        print('pprev-prev cloud num : ', len(self.pprev_prev_cloud))
        print('prev-current cloud num : ', len(self.prev_current_cloud))

        ratios = []
        for i in range(len(self.prev_current_cloud)):
            if i > 0:
                current_Xk1 = self.prev_current_cloud[i]
                current_Xk2 = self.prev_current_cloud[i-1]

                prev_Xk1 = self.pprev_prev_cloud[i]
                prev_Xk2 = self.pprev_prev_cloud[i-1]

                if (np.linalg.norm(current_Xk1 - current_Xk2) != 0):
                    ratios.append(np.linalg.norm(prev_Xk1 - prev_Xk2) / np.linalg.norm(current_Xk1 - current_Xk2))

        T_relative_scale = np.median(ratios)
        #T_relative_scale = np.mean(ratios)

        print('Relative Scale for Translation : ', T_relative_scale)

        absolute_scale = np.linalg.norm(np.array(self.ground_truth_T[self.dataset_current_idx]) - np.array(self.ground_truth_T[self.dataset_current_idx-1]))
        print('Absolute Sclae for Translation : ', absolute_scale)
        
        # Apply Forward Dominant Motoin Model with Absolute Value Comparison. 
        # This is implemented to prevent the error of accumulating Translation and Rotation when the camera is stationary, but the image is changing.
        # This condition is used to prevent nearby moving object's feature keypoint changes from affecting the calculation of translation and rotation.
        # Absolute value of Z is compared to absolute value of Y and X. Absolute value is used to make this condition work under both foward and backward movements of the camera.
        if ( (abs(self.geometric_unit_changes['T_prev_current'][2]) > abs(self.geometric_unit_changes['T_prev_current'][0])) and
             (abs(self.geometric_unit_changes['T_prev_current'][2]) > abs(self.geometric_unit_changes['T_prev_current'][1])) ):
            self.pose_T = self.prev_pose_T + T_relative_scale * self.prev_pose_R.dot(self.geometric_unit_changes['T_prev_current'])
            self.pose_R = self.geometric_unit_changes['R_prev_current'].dot(self.prev_pose_R)

            print('[INFO] Dominant Forward : Pose Estimation Results')
            print(self.pose_T)
            print(self.pose_R)

            return self.pose_T, self.pose_R

    def optimizePose_bundle_adjustment(self):
        # Local Bundle Optimization (Only between 2 camera poses)

        # Bundle Adjustment Optimizer
        BA_optimizer = BundleAdjustment()

        baseline = np.linalg.norm(self.prev_pose_T - self.pose_T)
        #baseline = 0.0

        BA_optimizer.add_pose(0, g2o.Quaternion(self.prev_pose_R), self.prev_pose_T.ravel(), self.fx, self.fy, self.cx, self.cy, baseline)
        BA_optimizer.add_pose(1, g2o.Quaternion(self.pose_R), self.pose_T.ravel(), self.fx, self.fy, self.cx, self.cy, baseline)
        
        #for i in range(math.floor(len(prev_current_cloud)/5)):
        for i in range(len(self.prev_current_cloud)):
            BA_optimizer.add_point(i+2, self.prev_current_cloud[i])
        
        #for i in range(math.floor(len(common_current_keypoints)/5)):
        for i in range(len(self.img_features_buffer[1])):
            BA_optimizer.add_edge(point_id=i+2, pose_id=0, measurement=self.img_features_buffer[1][i][0])
            BA_optimizer.add_edge(point_id=i+2, pose_id=1, measurement=self.img_features_buffer[2][i][0])

        BA_optimizer.optimize(max_iteration=100)
        
        return BA_optimizer.get_pose(1).translation().T, BA_optimizer.get_pose(1).rotation().R

    def update(self):

        # Update current values as prev values
        self.image_buffer[0] = self.image_buffer[1]
        self.img_features_buffer[0] = self.img_features_buffer[1]

        self.image_buffer[1] = self.image_buffer[2]
        self.img_features_buffer[1] = self.img_features_buffer[2]

        self.geometric_unit_changes['R_pprev_prev'] = self.geometric_unit_changes['R_prev_current']
        self.geometric_unit_changes['T_pprev_prev'] = self.geometric_unit_changes['T_prev_current']

        self.pprev_prev_cloud = self.prev_current_cloud

        self.prev_pose_T = self.pose_T
        self.prev_pose_R = self.pose_R
