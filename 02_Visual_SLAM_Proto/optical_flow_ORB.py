import numpy as np
import cv2 as cv

feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

cap = cv.VideoCapture('./slow_traffic_small.mp4')

color = np.random.randint(0, 255, (2000, 3))

orb = cv.ORB_create()

ret, old_frame = cap.read()
old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)

old_keypoints = orb.detect(old_gray, None)
old_keypoints, old_descriptors = orb.compute(old_gray, old_keypoints)
old_keypoints = np.array((old_keypoints))

### Convert ORB keypoints into numpy format (single point precision / column keypoint vector) ###########
correct_keypoints = []
for i in range(len(old_keypoints)):
    correct_keypoints.append([[np.float32(old_keypoints[i].pt[0]), np.float32(old_keypoints[i].pt[1])]])

np_old_correct_keypoints = np.array(correct_keypoints)
#########################################################################################################

print(type(np_old_correct_keypoints))
print(np_old_correct_keypoints)    

p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
print(type(p0))
print(p0)

mask = np.zeros_like(old_frame)

while(1):

    ret, frame = cap.read()
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    current_keypoints, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, np_old_correct_keypoints, None, **lk_params)

    good_new = current_keypoints[st==1]
    good_old = np_old_correct_keypoints[st==1]

    ### Feature Retracking : If the feature is below certain number, re-conduct feature extraction & tracking ####################
    print('Current Feature Num : ', len(good_new))
    if len(good_new) <= 400:

        print('[Re-Tracking] Current Feature Num : ', len(good_new))

        old_keypoints = orb.detect(old_gray, None)
        old_keypoints, old_descriptors = orb.compute(old_gray, old_keypoints)
        old_keypoints = np.array((old_keypoints))

        ### Convert ORB keypoints into numpy format (single point precision / column keypoint vector) #############################
        correct_keypoints = []
        for i in range(len(old_keypoints)):
            correct_keypoints.append([[np.float32(old_keypoints[i].pt[0]), np.float32(old_keypoints[i].pt[1])]])

        np_old_correct_keypoints = np.array(correct_keypoints)
        ###########################################################################################################################
        current_keypoints, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, np_old_correct_keypoints, None, **lk_params)
        good_new = current_keypoints[st==1]
        good_old = np_old_correct_keypoints[st==1]
    ################################################################################################################################

    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv.line(mask, (a, b), (c, d), color[i].tolist(), 2)
        frame = cv.circle(frame, (a, b), 5, color[i].tolist(), -1)

    img = cv.add(frame.copy(), mask)

    cv.imshow('ORB-based Optical Flow + ReTracking', img)
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break

    old_gray = frame_gray.copy()
    np_old_correct_keypoints = good_new.reshape(-1, 1, 2)
