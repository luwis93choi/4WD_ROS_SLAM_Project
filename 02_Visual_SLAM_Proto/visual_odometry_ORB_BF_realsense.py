import cv2 as cv
import numpy as np
import os
import sys

import pyrealsense2 as rs

from matplotlib import pyplot as plt
from matplotlib import gridspec
from mpl_toolkits.mplot3d import Axes3D

from time import sleep

class mono_visual_odom:

    def __init__(self):

        self.image_buffer = []   # image_buffer : [pprev_image, prev_image, current_image]
                                # ==> Store 3 consecutive images for translation rescaling

        self.img_features_buffer = [{'keypoints' : None, 'descriptors' : None}, 
                                    {'keypoints' : None, 'descriptors' : None}, 
                                    {'keypoints' : None, 'descriptors' : None}]
        self.common_img_features = []
        self.common_3D_clouds = []

        # Intel Realsense image pipeline initialization
        self.realsense_pipeline = rs.pipeline()

        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.profile = self.realsense_pipeline.start(self.config)

        # Camera intrinsic matrix params for 2D-3D triangulation
        # Acquiring Realsense Factory Default Intrinsic Parameters : https://github.com/IntelRealSense/librealsense/issues/2930
        self.intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.focal_length = self.intr.fx
        self.fx = self.intr.fx
        self.fy = self.intr.fy
        self.cx = self.intr.ppx
        self.cy = self.intr.ppy

        self.intrinsic_CAM_Mat = np.array([[self.fx, 0, self.cx],
                                           [0, self.fy, self.cy],
                                           [0, 0, 1]])

        print('focal length : ', self.focal_length)
        print('fx : ', self.intr.fx)
        print('fy : ', self.intr.fy)
        print('ppx : ', self.intr.ppx)
        print('ppy : ', self.intr.ppy)

        # ORB Feature Extractor
        self.orb = cv.ORB_create()

        # Brute Force Matcher
        self.BF_Matcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

        self.min_matches_num = 8

        self.pose_T = np.array([[0],
                                [0],
                                [0]])

        self.pose_R = np.array([[1, 0, 0],
                                [0, 1, 0],
                                [0, 0, 1]])

    def load_realsense_frames(self, _image_buffer, _image_pipeline):

        # Initialization : Load first 3 image frames into the buffer
        if len(_image_buffer) < 3:

            # Load 1st image frame into the image buffer
            frames = _image_pipeline.wait_for_frames()
            frame_pprev = frames.get_color_frame()
            image_pprev = np.asanyarray(frame_pprev.get_data())
            _image_buffer.append(image_pprev)

            print('[INFO] Loaded 1st image frame into the image buffer')
            
            # Load 2nd image frame into the image buffer
            frames = _image_pipeline.wait_for_frames()
            frame_prev = frames.get_color_frame()
            image_prev = np.asanyarray(frame_prev.get_data())
            _image_buffer.append(image_prev)

            print('[INFO] Loaded 2nd image frame into the image buffer')
        
            # Load 3rd image frame into the image buffer
            frames = _image_pipeline.wait_for_frames()
            frame_current = frames.get_color_frame()
            image_current = np.asanyarray(frame_current.get_data())
            _image_buffer.append(image_current)

            print('[INFO] Loaded 3rd image frame into the image buffer')
        
        # While visual odometry process is running, update the image buffer with the latest image frame
        else:

            # Push out the oldest image frame from the image buffer
            _image_buffer[0] = _image_buffer[1]
            _image_buffer[1] = _image_buffer[2]

            # Push the latest image frame into the image buffer
            frames = _image_pipeline.wait_for_frames()
            frame_current = frames.get_color_frame()
            image_current = np.asanyarray(frame_current.get_data())
            _image_buffer[2] = image_current

            print('[INFO] Updated the image buffer with the latest image frame')

        return _image_buffer

    def img_buffer_feature_extraction(self, _image_buffer, _image_feature_buffer, _feature_extractor):

        print('[1st Image Frame Process]')
        pprev_keypoints = _feature_extractor.detect(_image_buffer[0], None)
        pprev_keypoints, pprev_descriptors = _feature_extractor.compute(_image_buffer[0], pprev_keypoints)
        _image_feature_buffer[0]['keypoints'] = pprev_keypoints
        _image_feature_buffer[0]['descriptors'] = pprev_descriptors
        print('[INFO] pprev Image ORB Extraction Results - Feature Num : ', len(pprev_keypoints))

        print('[2nd Image Frame Process]')
        prev_keypoints = _feature_extractor.detect(_image_buffer[1], None)
        prev_keypoints, prev_descriptors = _feature_extractor.compute(_image_buffer[1], prev_keypoints)
        _image_feature_buffer[1]['keypoints'] = prev_keypoints
        _image_feature_buffer[1]['descriptors'] = prev_descriptors
        print('[INFO] prev Image ORB Extraction Results - Feature Num : ', len(prev_keypoints))


        print('[3rd Image Frame Process]')
        current_keypoints = _feature_extractor.detect(_image_buffer[2], None)
        current_keypoints, current_descriptors = _feature_extractor.compute(_image_buffer[2], current_keypoints)
        _image_feature_buffer[2]['keypoints'] = current_keypoints
        _image_feature_buffer[2]['descriptors'] = current_descriptors
        print('[INFO] current Image ORB Extraction Results - Feature Num : ', len(current_keypoints))

        return _image_feature_buffer

    def img_buffer_feature_matching(self, _image_feature_buffer, _feature_matcher):

        common_feature_buffer = [{'keypoints' : None, 'keypoints_pts' : None}, 
                                {'keypoints' : None, 'keypoints_pts' : None}, 
                                {'keypoints' : None, 'keypoints_pts' : None}]

        ### Filtering common feature keypoints by comparing the descriptors from 3 consecutive images ###
        pprev_feature_idx = []
        prev_feature_idx = []
        current_feature_idx = []

        pprev_prev_feature_matches = _feature_matcher.match(_image_feature_buffer[0]['descriptors'], _image_feature_buffer[1]['descriptors'])
        pprev_prev_feature_matches = sorted(pprev_prev_feature_matches, key=lambda x : x.distance)
        print('[INFO] pprev-prev ORB Feature Match Num : ', len(pprev_prev_feature_matches))

        #print(type(_image_feature_buffer[0]['descriptors']))
        #print(type(_image_feature_buffer[1]['descriptors']))

        pprev_prev_match_descriptors = []
        pprev_match_keypoints = []
        prev_match_keypoints = []
        current_match_keypoints = []

        for m in pprev_prev_feature_matches:

            pprev_feature_idx.append(m.queryIdx)
            prev_feature_idx.append(m.trainIdx)

            pprev_prev_match_descriptors.append(_image_feature_buffer[1]['descriptors'][m.trainIdx])

        # Datatype conversion from list to np.ndarray
        pprev_prev_match_descriptors = np.array(pprev_prev_match_descriptors)

        #print(type(pprev_prev_match_descriptors))

        pprev_prev_current_feature_matches = _feature_matcher.match(pprev_prev_match_descriptors, _image_feature_buffer[2]['descriptors'])
        pprev_prev_current_feature_matches = sorted(pprev_prev_current_feature_matches, key=lambda x : x.distance)

        print('[INFO] pprev-prev-current ORB Feature Match Num : ', len(pprev_prev_current_feature_matches))

        for m in pprev_prev_current_feature_matches:

            current_match_keypoints.append(_image_feature_buffer[2]['keypoints'][m.trainIdx])

            prev_match_keypoints.append(_image_feature_buffer[1]['keypoints'][prev_feature_idx[m.queryIdx]])

            pprev_match_keypoints.append(_image_feature_buffer[0]['keypoints'][pprev_feature_idx[m.queryIdx]])

        pprev_match_keypoints_pts = []
        prev_match_keypoints_pts = []
        current_match_keypoints_pts = []

        for m in pprev_prev_current_feature_matches:

            current_match_keypoints_pts.append(_image_feature_buffer[2]['keypoints'][m.trainIdx].pt)

            prev_match_keypoints_pts.append(_image_feature_buffer[1]['keypoints'][prev_feature_idx[m.queryIdx]].pt)

            pprev_match_keypoints_pts.append(_image_feature_buffer[0]['keypoints'][pprev_feature_idx[m.queryIdx]].pt)

        common_feature_buffer[0]['keypoints'] = pprev_match_keypoints
        common_feature_buffer[0]['keypoints_pts'] = pprev_match_keypoints_pts

        common_feature_buffer[1]['keypoints'] = prev_match_keypoints
        common_feature_buffer[1]['keypoints_pts'] = prev_match_keypoints_pts

        common_feature_buffer[2]['keypoints'] = current_match_keypoints
        common_feature_buffer[2]['keypoints_pts'] = current_match_keypoints_pts

        print('current_match_keypoints : ', len(current_match_keypoints))
        print('prev_match_keypoints : ', len(prev_match_keypoints))
        print('pprev_match_keypoints : ', len(pprev_match_keypoints))

        return common_feature_buffer

    def geometric_change_calc(self, _common_feature_buffer):
    
        geometric_unit_changes = {'R_pprev_prev' : None, 'T_pprev_prev' : None, 
                                'R_prev_current' : None, 'T_prev_current' : None}

        pprev_match_keypoints = _common_feature_buffer[0]['keypoints']
        pprev_match_keypoints_pts = _common_feature_buffer[0]['keypoints_pts']

        prev_match_keypoints = _common_feature_buffer[1]['keypoints']
        prev_match_keypoints_pts = _common_feature_buffer[1]['keypoints_pts']

        current_match_keypoints = _common_feature_buffer[2]['keypoints']
        current_match_keypoints_pts = _common_feature_buffer[2]['keypoints_pts']

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

        geometric_unit_changes['R_pprev_prev'] = Rotation_Mat_pprev_prev
        geometric_unit_changes['T_pprev_prev'] = Translation_Mat_pprev_prev
        geometric_unit_changes['R_prev_current'] = Rotation_Mat_prev_current
        geometric_unit_changes['T_prev_current'] = Translation_Mat_prev_current

        print('[R_pprev_prev]')
        print(Rotation_Mat_pprev_prev)
        print('[T_pprev_prev]')
        print(Translation_Mat_pprev_prev)
        print('[R_prev_current]')
        print(Rotation_Mat_prev_current)
        print('[T_prev_current]')
        print(Translation_Mat_prev_current)

        return geometric_unit_changes

    def img_common3Dcloud_triangulate(self, _common_feature_buffer, _geometric_unit_changes):

        pprev_match_keypoints_pts = np.float32(_common_feature_buffer[0]['keypoints_pts']).reshape(2, -1)
        prev_match_keypoints_pts = np.float32(_common_feature_buffer[1]['keypoints_pts']).reshape(2, -1)
        current_match_keypoints_pts = np.float32(_common_feature_buffer[2]['keypoints_pts']).reshape(2, -1)

        Rotation_Mat_pprev_prev = _geometric_unit_changes['R_pprev_prev']
        Translation_Mat_pprev_prev = _geometric_unit_changes['T_pprev_prev']
        Rotation_Mat_prev_current = _geometric_unit_changes['R_prev_current']
        Translation_Mat_prev_current = _geometric_unit_changes['T_prev_current']

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

        pprev_prev_cloud = cv.triangulatePoints(P0, P1, pprev_match_keypoints_pts, prev_match_keypoints_pts).reshape(-1, 4)[:, :3]

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

        prev_current_cloud = cv.triangulatePoints(P0, P1, prev_match_keypoints_pts, current_match_keypoints_pts).reshape(-1, 4)[:, :3]

        return pprev_prev_cloud, prev_current_cloud

    def pose_estimate(self, _pose_T, _pose_R, _prev_current_Translation_Mat, _prev_current_Rotation_Mat, _pprev_prev_cloud, _prev_current_cloud):

        print('pprev-prev cloud num : ', len(_pprev_prev_cloud))
        print('prev-current cloud num : ', len(_prev_current_cloud))

        ratios = []
        for i in range(len(_prev_current_cloud)):
            if i > 0:
                current_Xk1 = _prev_current_cloud[i]
                current_Xk2 = _prev_current_cloud[i-1]

                prev_Xk1 = _pprev_prev_cloud[i]
                prev_Xk2 = _pprev_prev_cloud[i-1]

                if (np.linalg.norm(current_Xk1 - current_Xk2) != 0):
                    ratios.append(np.linalg.norm(prev_Xk1 - prev_Xk2) / np.linalg.norm(current_Xk1 - current_Xk2))

        T_relative_scale = np.median(ratios)

        print('Relative Scale for Translation : ', T_relative_scale)

        # Apply Forward Dominant Motoin Model with Absolute Value Comparison. 
        # This is implemented to prevent the error of accumulating Translation and Rotation when the camera is stationary, but the image is changing.
        # This condition is used to prevent nearby moving object's feature keypoint changes from affecting the calculation of translation and rotation.
        # Absolute value of Z is compared to absolute value of Y and X. Absolute value is used to make this condition work under both foward and backward movements of the camera.
        if (abs(_prev_current_Translation_Mat[2]) > abs(_prev_current_Translation_Mat[0])) and (abs(_prev_current_Translation_Mat[2]) > abs(_prev_current_Translation_Mat[1])):
            _pose_T = _pose_T + T_relative_scale * _pose_R.dot(_prev_current_Translation_Mat)
            _pose_R = _prev_current_Rotation_Mat.dot(_pose_R)

        print('[INFO] Pose Estimation Results')
        print(_pose_T)

        return _pose_T, _pose_R

    def frame_Skip(self, _common_feature_buffer):

        pprev_match_keypoints_pts = np.float32(_common_feature_buffer[0]['keypoints_pts'])
        prev_match_keypoints_pts = np.float32(_common_feature_buffer[1]['keypoints_pts'])
        current_match_keypoints_pts = np.float32(_common_feature_buffer[2]['keypoints_pts'])

        pixel_diff = np.mean(abs(current_match_keypoints_pts - prev_match_keypoints_pts))

        print('[Pixel DIFF] : ',  pixel_diff)

        return pixel_diff < 3

    def optimize_bundle_adjustment(self):

        print()

    def calc_trajectory(self):

        self.load_realsense_frames(self.image_buffer, self.realsense_pipeline)

        self.img_buffer_feature_extraction(self.image_buffer, self.img_features_buffer, self.orb)

        common_features = self.img_buffer_feature_matching(self.img_features_buffer, self.BF_Matcher)

        if(self.frame_Skip(common_features) == False):

            # If there are enough common feature keypoints, compute geometric changes over 3 images and estimate camera pose.
            if(len(common_features[0]['keypoints']) >= self.min_matches_num):

                geometric_unit_changes = self.geometric_change_calc(common_features)

                pprev_prev_cloud, prev_current_cloud = self.img_common3Dcloud_triangulate(common_features, geometric_unit_changes)

                self.pose_T, self.pose_R = self.pose_estimate(self.pose_T, self.pose_R, geometric_unit_changes['T_prev_current'], geometric_unit_changes['R_prev_current'], pprev_prev_cloud, prev_current_cloud)
                
            # If there are not enough commone keypoints for Essential Matrix Decomposition, skip the current frame and load a new image.
            else:

                print('[FRAME SKIPPED] : Not Enough Feature Keypoints')

        else:

                print('[FRAME SKIPPED] : Camera is stationary / No need to accumulate pose data')

        print('----------------------------------------------------------------')

        return self.pose_T, self.pose_R