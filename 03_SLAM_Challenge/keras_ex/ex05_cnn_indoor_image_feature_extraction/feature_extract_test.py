# Reference 1 : https://medium.com/@aybukeyalcinerr/bag-of-visual-words-bovw-db9500331b2f
# Reference 2 : https://gurus.pyimagesearch.com/the-bag-of-visual-words-model/
# Reference 3 : https://bskyvision.com/475

from __future__ import print_function

import matplotlib.pyplot as plt

import numpy as np
import cv2
import os
import sys

from scipy import ndimage
from scipy.spatial import distance
from sklearn.cluster import KMeans
from sklearn.cluster import MiniBatchKMeans

import argparse

# Construct the argument parser
ap = argparse.ArgumentParser()
ap.add_argument('-d', '--dataset', type=str, required=True,
    help='Path to the dataset')
ap.add_argument('-m', '--mode', type=str, required=True,
    help='Select mode for feature extraction')
args = vars(ap.parse_args())

# Read and display image data from the designated dataset path
image_dataset = sorted(os.listdir(args['dataset']))

print(len(image_dataset))

# Mode 1 : ORB Feature Extraction
if args['mode'] is '1':

    orb = cv2.ORB_create()

    for img_path in image_dataset:

        current_img = cv2.imread(args['dataset'] + '/' + img_path)

        resized_img = cv2.resize(current_img, dsize=(300, 300), interpolation=cv2.INTER_AREA)

        keypoints = orb.detect(resized_img, None)
        keypoints, descriptors = orb.compute(resized_img, keypoints)

        output_img = resized_img
        cv2.drawKeypoints(resized_img, keypoints, output_img, color=(0, 255, 0), flags=0)

        key_test = np.array(keypoints)
        des_test = np.array(descriptors)

        print(key_test.shape)
        print(des_test.shape)
        print('-----------------------------------')

        cv2.imshow('test', output_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

# Mode 2 : Visaul Bag of Words Feature Extraction
elif args['mode'] is '2':

    # Building a visual dictionary of the entire image dataset
    image_list = []
    descriptors_list = []
    keypoints_list = []

    orb = cv2.ORB_create()
    progress = 0
    total_dataset_num = float(len(image_dataset))
    for img_path in image_dataset:

        current_img = cv2.imread(args['dataset'] + '/' + img_path)

        resized_img = cv2.resize(current_img, dsize=(300, 300), interpolation=cv2.INTER_AREA)

        keypoints = orb.detect(resized_img, None)
        keypoints, descriptors = orb.compute(resized_img, keypoints)

        image_list.append(resized_img)
        #descriptors_list.append(descriptors)
        keypoints_list.append(keypoints)

        if descriptors is not None:
            for des in descriptors:
                descriptors_list.append(des)

        progress += 1

        print('Processing [' + '{:.2f}'.format(float(progress)/total_dataset_num*100) + '%]', end='\r')
        sys.stdout.flush()

    #visual_dict = {'img' : image_list, 'descriptors' : descriptors_list, 'keypoints' : keypoints_list}

    # Send the visual dictionary to k-means clustering algorithm
    # Find the visual words which are center points
    kmeans = KMeans(n_clusters=150, n_init=10)
    kmeans.fit(descriptors_list)
    visual_words = kmeans.cluster_centers_
