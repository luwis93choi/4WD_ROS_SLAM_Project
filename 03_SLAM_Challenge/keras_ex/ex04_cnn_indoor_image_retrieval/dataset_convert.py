from __future__ import print_function

import os
import sys
import cv2

import numpy as np
import argparse

import pickle

import random
import math

from sklearn.model_selection import train_test_split

# Construct the argument parser to receive the path to the dataset
ap = argparse.ArgumentParser()
ap.add_argument('-d', '--dataset', type=str, required=True,
    help='Path to the dataset')
ap.add_argument('-o', '--output_img_mat_list', type=str, required=True,
    help='Path to the output image matrix list')
ap.add_argument('-t', '--train_data_ratio', type=float, required=True,
    help='Ratio of training data among the entire dataset (Betwen 0 and 1)')
args = vars(ap.parse_args())

print('Accessing dataset path : ' + args['dataset'])

# Read and display image data from the designated dataset path
image_dataset = sorted(os.listdir(args['dataset']))

print(len(image_dataset))

total_dataset_num = float(len(image_dataset))

progress = 0
gray_img_mat_set = []
for img_path in image_dataset:

    img = cv2.imread(args['dataset'] + '/' + img_path)
    resized_img = cv2.resize(img, dsize=(300, 300), interpolation=cv2.INTER_AREA)

    gray_img = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)

    gray_img_mat_set.append(np.array(gray_img))

    progress += 1

    print('Reading [' + '{:.2f}'.format(float(progress)/total_dataset_num*100) + '%]', end='\r')
    sys.stdout.flush()

print('Reading Dataset Completed')

print('Training data ratio : ' + str(float(args['train_data_ratio'])))

training_data_num = int(math.floor(total_dataset_num * float(args['train_data_ratio'])))
print('Training data num : ' + str(training_data_num))

print('Testing data ratio : ' + str(1 - float(args['train_data_ratio'])))

testing_data_num = int(total_dataset_num) - training_data_num
print('Training data num : ' + str(testing_data_num))

gray_dataset_train = []
gray_dataset_test = []

gray_dataset_train, gray_dataset_test = train_test_split(gray_img_mat_set, test_size=(1-float(args['train_data_ratio'])), shuffle=True)

#gray_dataset_train = random.sample(gray_img_mat_set, training_data_num)
#gray_dataset_test = random.sample(gray_img_mat_set, testing_data_num)
gray_dataset = {"gray_mat_train" : gray_dataset_train, "gray_mat_test" : gray_dataset_test}

cv2.imshow('train_img', gray_dataset["gray_mat_train"][0])
cv2.imshow('test_img', gray_dataset["gray_mat_test"][1])
cv2.waitKey(0)
cv2.destroyAllWindows()

f = open(args['output_img_mat_list'], 'wb')
f.write(pickle.dumps(gray_dataset))
f.close()