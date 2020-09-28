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
focal_length = 1.93
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
'''
pose_T = np.array([[0],
                   [0],
                   [0]])

pose_R = np.array([[1, 0, 0],
                   [0, 1, 0],
                   [0, 0, 1]])
'''

match_img_1 = None
match_img_2 = None

pose_T = None
pose_R = None

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
    current_keypoints = _feature_extractor.detect(_image_buffer[0], None)
    current_keypoints, current_descriptors = _feature_extractor.compute(_image_buffer[2], current_keypoints)
    _image_feature_buffer[2]['keypoints'] = current_keypoints
    _image_feature_buffer[2]['descriptors'] = current_descriptors
    print('[INFO] current Image ORB Extraction Results - Feature Num : ', len(current_keypoints))

    return _image_feature_buffer

def img_buffer_feature_matching(_image_feature_buffer, _feature_matcher):

    common_feature_buffer = [{'keypoints' : None, 'descriptors' : None}, 
                             {'keypoints' : None, 'descriptors' : None}, 
                             {'keypoints' : None, 'descriptors' : None}]
 
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

    print('current_match_keypoints : ', len(current_match_keypoints))
    print('prev_match_keypoints : ', len(prev_match_keypoints))
    print('pprev_match_keypoints : ', len(pprev_match_keypoints))

    ### Draw matches over 3 consecutive images ################################
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

    cv.imshow('Images with common feature points [ppev, prev, current]', full_view)    
    cv.waitKey(0)
    ##############################################################################

    return common_feature_buffer

'''
def img_common3Dcloud_triangulate():

def pose_estimate():
'''

while True:

    load_realsense_frames(image_buffer, realsense_pipeline)

    img_buffer_feature_extraction(image_buffer, img_features_buffer, orb)

    img_buffer_feature_matching(img_features_buffer, BF_Matcher)
    '''
    full_view = np.hstack((image_buffer[0], image_buffer[1], image_buffer[2]))
    
    cv.imshow('Images in the buffer [pprev, prev, current]', full_view)
    
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break
    '''
'''
def processFirstFrame(pprev_image, feature_extractor):

    print('[1st Image Frame Process]')

    pprev_keypoints = feature_extractor.detect(pprev_image, None)
    pprev_keypoints, pprev_descriptors = feature_extractor.compute(pprev_image, pprev_keypoints)

    print('[INFO] pprev Image ORB Extraction Results - Feature Num : ', len(pprev_keypoints))

    return pprev_keypoints, pprev_descriptors

def processSecondFrame(pprev_image, prev_image, pprev_keypoints, pprev_descriptors, feature_extractor, feature_matcher):

    print('[2nd Image Frame Process]')

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

    pprev_track_pts_int = np.int32(pprev_track_pts)
    prev_track_pts_int = np.int32(prev_track_pts)

    # Compute Essential Matrix between pprev image and prev image 
    # in order to acquire the translation and rotation between t-2 and t-1 images
    Essential_Mat, mask = cv.findEssentialMat(pprev_track_pts_int, prev_track_pts_int, 
                                              focal=focal_length, pp=(cx, cy),
                                              method=cv.FM_RANSAC, prob=0.999, threshold=1.0)

    print('[INFO] pprev-prev Essential Matrix')
    print(Essential_Mat)

    retval, Rotation_Mat, Translation_Mat, mask = cv.recoverPose(Essential_Mat, pprev_track_pts_int, prev_track_pts_int, 
                                                                 focal=focal_length, pp=(cx, cy))

    print('[INFO] pprev-prev Translation Matrix (Unscaled)')
    print(Translation_Mat)

    print('[INFO] pprev-prev Rotation Matrix')
    print(Rotation_Mat)
    
    extrinsic_CAM_Mat = np.hstack((Rotation_Mat, Translation_Mat))

    pprev_track_pts_tri = np.float32(pprev_track_pts).reshape(2, -1)
    prev_track_pts_tri = np.float32(prev_track_pts).reshape(2, -1)

    # The canonical matrix (set as the origin)
    P0 = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0]])
    P0 = intrinsic_CAM_Mat.dot(P0)
    # Rotated and translated using P0 as the reference point
    P1 = np.hstack((Rotation_Mat, Translation_Mat))
    P1 = intrinsic_CAM_Mat.dot(P1)

    # Use 2D -> 3D Triangulation in order to acquire 3D point cloud image between pprev image and prev image
    prev_cloud = cv.triangulatePoints(P0, P1, pprev_track_pts_tri, prev_track_pts_tri).reshape(-1, 4)[:, :3]

    return pprev_track_pts, pprev_track_des, prev_track_pts, prev_track_des, prev_cloud, Translation_Mat, Rotation_Mat

def processThirdFrame(prev_image, current_image, prev_keypoints, prev_descriptors, feature_extractor, feature_matcher):

    print('[3rd Image Frame Process]')

    current_keypoints = feature_extractor.detect(current_image)
    current_keypoints, current_descriptors = feature_extractor.compute(current_image, current_keypoints)

    print('[INFO] current Image ORB Extraction Results - Feature Num : ', len(current_keypoints))

    current_feature_matches = feature_matcher.match(cv.UMat(np.array(prev_descriptors, dtype=np.uint8)), current_descriptors)
    current_feature_matches = sorted(current_feature_matches, key=lambda x:x.distance)

    print('[INFO] prev-current ORB Feature Match Num : ', len(current_feature_matches))

    # Compile the keypoints and descriptors from feature matching results
    prev_track_pts = []
    current_track_pts = []

    prev_track_des = []
    current_track_des = []

    for m in current_feature_matches:
        prev_track_pts.append(prev_keypoints[m.queryIdx])
        current_track_pts.append(current_keypoints[m.trainIdx].pt)

        prev_track_des.append(prev_descriptors[m.queryIdx])
        current_track_des.append(current_descriptors[m.trainIdx])

    prev_track_pts_int = np.int32(prev_track_pts)
    current_track_pts_int = np.int32(current_track_pts)

    # Compute Essential Matrix between pprev image and prev image 
    # in order to acquire the translation and rotation between t-2 and t-1 images
    Essential_Mat, mask = cv.findEssentialMat(prev_track_pts_int, current_track_pts_int, 
                                              focal=focal_length, pp=(cx, cy),
                                              method=cv.FM_RANSAC, prob=0.999, threshold=1.0)

    print('[INFO] prev-current Essential Matrix')
    print(Essential_Mat)

    retval, Rotation_Mat, Translation_Mat, mask = cv.recoverPose(Essential_Mat, prev_track_pts_int, current_track_pts_int, 
                                                                 focal=focal_length, pp=(cx, cy))

    print('[INFO] prev-current Translation Matrix (Unscaled)')
    print(Translation_Mat)

    print('[INFO] prev-current Rotation Matrix')
    print(Rotation_Mat)
    
    extrinsic_CAM_Mat = np.hstack((Rotation_Mat, Translation_Mat))

    prev_track_pts_tri = np.float32(prev_track_pts).reshape(2, -1)
    current_track_pts_tri = np.float32(current_track_pts).reshape(2, -1)

    # The canonical matrix (set as the origin)
    P0 = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0]])
    P0 = intrinsic_CAM_Mat.dot(P0)
    # Rotated and translated using P0 as the reference point
    P1 = np.hstack((Rotation_Mat, Translation_Mat))
    P1 = intrinsic_CAM_Mat.dot(P1)

    # Use 2D -> 3D Triangulation in order to acquire 3D point cloud image between prev image and current image
    current_cloud = cv.triangulatePoints(P0, P1, prev_track_pts_tri, current_track_pts_tri).reshape(-1, 4)[:, :3]
    
    return prev_track_pts, prev_track_des, current_track_pts, current_track_des, current_cloud, Translation_Mat, Rotation_Mat

def pose_estimate(pose_T, pose_R, current_Translate_Mat, current_Rotate_Mat, prev_3D_cloud, current_3D_cloud):

    print('[Pose estimation using Relative Scale along 3 consecutive images]')

    # Relative scale computation using the distance between 3-D points of tracking features
    print('prev_cloud shape : ', prev_3D_cloud.shape)
    print('current_cloud shape : ', current_3D_cloud.shape)

    # Between two 3-D points of tracking features from t image, t-1 image, t-2 image, 
    # select the common 3-D points
    common_cloud_num = min([prev_3D_cloud.shape[0], current_3D_cloud.shape[0]])
    print('Number of common 3-D features points : ', common_cloud_num)

    # Process common 3-D points for relative scaling
    ratios = []
    for i in range(common_cloud_num):
        if i > 0:   # Pick a pair of common 3-D points and use them to calculate the depth (disparsity)
            
            # A pair of common 3-D points from current clouds
            current_Xk1 = current_3D_cloud[i]
            current_Xk2 = current_3D_cloud[i-1]

            # A pair of common 3-D points from previous clouds
            prev_Xk1 = prev_3D_cloud[i]
            prev_Xk2 = prev_3D_cloud[i-1]

            # Use np.linalg.norm to acquire L2 norm between a pair of common 3-D points, which serves as the distance between the pair
            if np.linalg.norm(current_Xk1 - current_Xk2 != 0):
                ratios.append(np.linalg.norm(prev_Xk1 - prev_Xk2) / np.linalg.norm(current_Xk1 - current_Xk2))

    T_relative_scale = np.median(ratios)    # Use median value as the final relative scale result

    print('Relative Scale for Translation : ', T_relative_scale)
    
    # Consider dominant forward motion
    # This is because when the vehicle is standing still, there should not be changes in feature coordinates.
    # As a result, when standing still, transformation has to be zero. Therefore, consider forward translation as dominant component in camera pose.
    if(current_Translate_Mat[2] > current_Translate_Mat[0]) and (current_Translate_Mat[2] > current_Translate_Mat[1]):
        pose_T = pose_T + T_relative_scale * pose_R.dot(current_Translate_Mat)
        pose_R = current_Rotate_Mat.dot(pose_R)

    print('[INFO] Pose Estimation Results')
    print(pose_T)

    return pose_T, pose_R

init_flag = True
plot_background = np.ones((300, 300))

plt.grid()

while True:

    load_realsense_frames(image_buffer, realsense_pipeline)

    pprev_keypoints, pprev_descriptors = processFirstFrame(image_buffer[0], orb)

    pprev_track_pts, pprev_track_des, prev_track_pts, prev_track_des, prev_cloud, prev_Translation_Mat, prev_Rotation_Mat = processSecondFrame(image_buffer[0], image_buffer[1], pprev_keypoints, pprev_descriptors, orb, BF_Matcher)

    if init_flag is True:

        print('--- Init Pose ---')
        pose_T = prev_Translation_Mat
        pose_R = prev_Rotation_Mat

        init_flag = False

    prev_track_pts, prev_track_des, current_track_pts, current_track_des, current_cloud, current_Translation_Mat, current_Rotation_Mat = processThirdFrame(image_buffer[1], image_buffer[2], prev_track_pts, prev_track_des, orb, BF_Matcher)

    pose_T, pose_R = pose_estimate(pose_T, pose_R, current_Translation_Mat, current_Rotation_Mat, prev_cloud, current_cloud)

    # Draw the trajectory 
    plt.xlim(-30.0, 30.0)
    plt.ylim(-30.0, 30.0)
    plt.scatter(pose_T[0][0], pose_T[2][0])

    plt.draw()
    plt.show(block=False)
    plt.pause(0.01)
    
    full_view = np.hstack((image_buffer[0], image_buffer[1], image_buffer[2]))
    
    cv.imshow('Images in the buffer [pprev, prev, current]', full_view)
    
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break

    print('-----------------------------------------------------')
'''