import cv2 as cv
import numpy as np
import os
import sys

import pyrealsense2 as rs

from matplotlib import pyplot as plt
from matplotlib import gridspec
from mpl_toolkits.mplot3d import Axes3D

from time import sleep

print('OpenCV Ver : ', cv.__version__)

image_buffer = []   # image_buffer : [pprev_image, prev_image, current_image]
                    # ==> Store 3 consecutive images for translation rescaling

img_features_buffer = [{'keypoints' : None, 'descriptors' : None}, 
                       {'keypoints' : None, 'descriptors' : None}, 
                       {'keypoints' : None, 'descriptors' : None}]
common_img_features = []
common_3D_clouds = []

# Intel Realsense image pipeline initialization
realsense_pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = realsense_pipeline.start(config)

# Camera intrinsic matrix params for 2D-3D triangulation
# Acquiring Realsense Factory Default Intrinsic Parameters : https://github.com/IntelRealSense/librealsense/issues/2930
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
focal_length = intr.fx
fx = intr.fx
fy = intr.fy
cx = intr.ppx
cy = intr.ppy

intrinsic_CAM_Mat = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0, 0,  1]])

print('focal length : ', focal_length)
print('fx : ', intr.fx)
print('fy : ', intr.fy)
print('ppx : ', intr.ppx)
print('ppy : ', intr.ppy)

# ORB Feature Extractor
orb = cv.ORB_create()

# Brute Force Matcher
BF_Matcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

top_max_matches_num = 10

# Params for Lucas-Kanade Optical Flow
lk_params = dict(winSize = (21, 21),
                 maxLevel = 3,
                 criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 30, 0.01))

pose_T = np.array([[0],
                   [0],
                   [0]])

pose_R = np.array([[1, 0, 0],
                   [0, 1, 0],
                   [0, 0, 1]])

def load_realsense_frames(_image_buffer, _image_pipeline):

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

def img_buffer_feature_extraction(_image_buffer, _image_feature_buffer, _feature_extractor):

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

def img_buffer_feature_matching(_image_feature_buffer, _feature_matcher):

    common_feature_buffer = [{'keypoints' : None, 'keypoints_pts' : None}, 
                             {'keypoints' : None, 'keypoints_pts' : None}, 
                             {'keypoints' : None, 'keypoints_pts' : None}]
 
    geometric_unit_changes = {'R_pprev_prev' : None, 'T_pprev_prev' : None, 
                              'R_prev_current' : None, 'T_prev_current' : None}

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

    ### Essential Matrix Calcuation & Rotation/Translation Matrix Calculation ###
    Essential_Mat_pprev_prev, mask_pprev_prev = cv.findEssentialMat(np.int32(prev_match_keypoints_pts), 
                                                                    np.int32(pprev_match_keypoints_pts),
                                                                    cameraMatrix=intrinsic_CAM_Mat,
                                                                    method=cv.RANSAC, prob=0.999, threshold=1.0)

    Essential_Mat_prev_current, mask_prev_current = cv.findEssentialMat(np.int32(current_match_keypoints_pts), 
                                                                        np.int32(prev_match_keypoints_pts),
                                                                        cameraMatrix=intrinsic_CAM_Mat,
                                                                        method=cv.RANSAC, prob=0.999, threshold=1.0)

    retval, Rotation_Mat_pprev_prev, Translation_Mat_pprev_prev, r_mask_pprev_prev = cv.recoverPose(Essential_Mat_pprev_prev,
                                                                                                    np.int32(prev_match_keypoints_pts),
                                                                                                    np.int32(pprev_match_keypoints_pts),
                                                                                                    cameraMatrix=intrinsic_CAM_Mat)

    retval, Rotation_Mat_prev_current, Translation_Mat_prev_current, r_mask_prev_current = cv.recoverPose(Essential_Mat_prev_current,
                                                                                                          np.int32(current_match_keypoints_pts),
                                                                                                          np.int32(prev_match_keypoints_pts),
                                                                                                          cameraMatrix=intrinsic_CAM_Mat)

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

    ### [Not Used] Common Feature Selection Optimization : Choose the common viable features without pixel coordinate jumps ###
    pprev_prev_feature_pixel_distance = []
    prev_current_feature_pixel_distance = []
    
    for i in range(len(pprev_prev_current_feature_matches)):
        pprev_prev_feature_pixel_distance.append(np.linalg.norm(np.array(pprev_match_keypoints[i].pt) - np.array(prev_match_keypoints[i].pt)))
        prev_current_feature_pixel_distance.append(np.linalg.norm(np.array(prev_match_keypoints[i].pt) - np.array(current_match_keypoints[i].pt)))

    #print('pprev_prev_feature_pixel_distance : ', pprev_prev_feature_pixel_distance)
    #print('prev_current_feature_pixel_distance : ', prev_current_feature_pixel_distance)

    pprev_prev_mean = np.mean(pprev_prev_feature_pixel_distance)
    prev_current_mean = np.mean(prev_current_feature_pixel_distance)

    ### Setup match result image with the common features over 3 consecutive images #########
    img_pprev = image_buffer[0].copy()
    img_prev = image_buffer[1].copy()
    img_curr = image_buffer[2].copy()
    
    for i in range(len(pprev_match_keypoints)):

        color_val = np.random.randint(256, size=3)
        img_pprev = cv.circle(img_pprev, tuple(np.int32(pprev_match_keypoints[i].pt)), 3, (int(color_val[0]), int(color_val[1]), int(color_val[2])), -1)
        img_prev = cv.circle(img_prev, tuple(np.int32(prev_match_keypoints[i].pt)), 3, (int(color_val[0]), int(color_val[1]), int(color_val[2])), -1)
        img_curr = cv.circle(img_curr, tuple(np.int32(current_match_keypoints[i].pt)), 3, (int(color_val[0]), int(color_val[1]), int(color_val[2])), -1)

    full_view = np.hstack((img_pprev, img_prev, img_curr))
    
    for i in range(len(pprev_match_keypoints)):

        full_view = cv.line(full_view, tuple([np.int32(pprev_match_keypoints[i].pt)[0], np.int32(pprev_match_keypoints[i].pt)[1]]), 
                                       tuple([np.int32(prev_match_keypoints[i].pt)[0] + 640, np.int32(prev_match_keypoints[i].pt)[1]]), (0, 0, 255), 1)
        full_view = cv.line(full_view, tuple([np.int32(prev_match_keypoints[i].pt)[0] + 640, np.int32(prev_match_keypoints[i].pt)[1]]), 
                                       tuple([np.int32(current_match_keypoints[i].pt)[0] + 640*2, np.int32(current_match_keypoints[i].pt)[1]]), (0, 0, 255), 1)

    '''
    ### Draw match result image with feature pixel distance distribution histogram #####
    grid = plt.GridSpec(nrows=2, ncols=2)
    
    graph_1 = plt.subplot(grid[0, 0])
    graph_1.hist(pprev_prev_feature_pixel_distance)
    graph_1.title.set_text('Feature Pixel Distance Distribution (pprev - prev)')
    graph_1.set_xlim(0, 800)
    graph_1.axvline(pprev_prev_mean, color='black', linestyle='dashed', linewidth=1)
    
    graph_2 = plt.subplot(grid[0, 1])
    graph_2.hist(prev_current_feature_pixel_distance)
    graph_2.title.set_text('Feature Pixel Distance Distribution (prev - current)')
    graph_2.set_xlim(0, 800)
    graph_2.axvline(prev_current_mean, color='black', linestyle='dashed', linewidth=1)
    
    graph_3 = plt.subplot(grid[1,0:])
    graph_3.imshow(full_view)
    graph_3.title.set_text('Feature Matching Results over 3 Consecutive Images')
    
    #plt.show()

    plt.draw()
    plt.show(block=False)
    plt.pause(0.001)
    '''
    return common_feature_buffer, geometric_unit_changes

def img_common3Dcloud_triangulate(_common_feature_buffer, _geometric_unit_changes):

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
    P0 = intrinsic_CAM_Mat.dot(P0)
    # Rotated and translated using P0 as the reference point
    P1 = np.hstack((Rotation_Mat_pprev_prev, Translation_Mat_pprev_prev))
    P1 = intrinsic_CAM_Mat.dot(P1)

    pprev_prev_cloud = cv.triangulatePoints(P0, P1, pprev_match_keypoints_pts, prev_match_keypoints_pts).reshape(-1, 4)[:, :3]

    print('[INFO] prev-current Triangulation')
    ### Triangluation between prev and current
    # The canonical matrix (set as the origin)
    P0 = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0]])
    P0 = intrinsic_CAM_Mat.dot(P0)
    # Rotated and translated using P0 as the reference point
    P1 = np.hstack((Rotation_Mat_prev_current, Translation_Mat_prev_current))
    P1 = intrinsic_CAM_Mat.dot(P1)

    prev_current_cloud = cv.triangulatePoints(P0, P1, prev_match_keypoints_pts, current_match_keypoints_pts).reshape(-1, 4)[:, :3]

    return pprev_prev_cloud, prev_current_cloud

def pose_estimate(_pose_T, _pose_R, _prev_current_Translation_Mat, _prev_current_Rotation_Mat, _pprev_prev_cloud, _prev_current_cloud):

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

    if (abs(_prev_current_Translation_Mat[2]) > abs(_prev_current_Translation_Mat[0])) and (abs(_prev_current_Translation_Mat[2]) > abs(_prev_current_Translation_Mat[1])):
        _pose_T = _pose_T + T_relative_scale * _pose_R.dot(_prev_current_Translation_Mat)
        _pose_R = _prev_current_Rotation_Mat.dot(_pose_R)

    print('[INFO] Pose Estimation Results')
    print(_pose_T)

    return _pose_T, _pose_R

while True:

    load_realsense_frames(image_buffer, realsense_pipeline)

    img_buffer_feature_extraction(image_buffer, img_features_buffer, orb)

    common_features, geometric_unit_changes = img_buffer_feature_matching(img_features_buffer, BF_Matcher)

    pprev_prev_cloud, prev_current_cloud = img_common3Dcloud_triangulate(common_features, geometric_unit_changes)

    pose_T, pose_R = pose_estimate(pose_T, pose_R, geometric_unit_changes['T_prev_current'], geometric_unit_changes['R_prev_current'], pprev_prev_cloud, prev_current_cloud)
    
    # Draw the trajectory 
    plt.xlim(-30.0, 30.0)
    plt.ylim(-30.0, 30.0)
    plt.scatter(pose_T[0][0], pose_T[2][0])

    #plt.draw()
    plt.pause(0.000001)
    plt.show(block=False)
       
    print('----------------------------------------------------------------')
