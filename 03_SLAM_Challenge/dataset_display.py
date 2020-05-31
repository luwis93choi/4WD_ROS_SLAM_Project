import os
import sys
import cv2

import numpy as np

import h5py

train_1f_dataset_path = './indoor_dataset/dataset/1f/train'
train_1f_dataset_date = sorted(os.listdir(train_1f_dataset_path))
train_1f_dataset_image_path = ''
train_1f_dataset_image_filelist = []
train_1f_dataset_pack_list = []
train_1f_camera_id_list = []
train_1f_dataset_image_list_by_ID = []

train_b1_dataset_path = './indoor_dataset/dataset/b1/train'

test_1f_dataset_path = './indoor_dataset/dataset/1f/test'

test_b1_dataset_path = './indoor_dataset/dataset/b1/test'

# Reading the list of dataset packages 
for i in range(len(train_1f_dataset_date)):

    train_1f_dataset_pack_list.append(train_1f_dataset_path + '/' + train_1f_dataset_date[i])

# Read and display image data from each dataset package
for i in range(len(train_1f_dataset_pack_list)):

    # Read camera parameter textfile in order to acquire the list of camera IDs
    print('Open camera parameters : ' + train_1f_dataset_pack_list[i])
    camera_param = open(train_1f_dataset_pack_list[i] + '/camera_parameters.txt', 'r')
    
    lines = camera_param.readlines()

    for j in range(len(lines)):

        index = lines[j].find(' ')

        # Only first word of each sentence that is longer than size of 2 is considered as camera ID
        if index > 2:
            
            #  When a camera ID value is identified from the parameter file, add it to ID list
            train_1f_camera_id_list.append(lines[j][0:index])

    camera_param.close()

    print('Current camera ID : ')
    print(train_1f_camera_id_list)
    print('')

    for j in range(len(train_1f_camera_id_list)):

        train_1f_dataset_image_list_by_ID.append([])

    print(train_1f_dataset_image_list_by_ID)

    train_1f_dataset_image_path = train_1f_dataset_pack_list[i] + '/images'
    print(train_1f_dataset_image_path)
    print('')

    train_1f_dataset_image_filelist = sorted(os.listdir(train_1f_dataset_image_path))

    for j in range(len(train_1f_dataset_image_filelist)):

        for k in range(len(train_1f_camera_id_list)):

            if train_1f_dataset_image_filelist[j].startswith(train_1f_camera_id_list[k]):
                
                train_1f_dataset_image_list_by_ID[k].append(train_1f_dataset_image_filelist[j])

    #print(train_1f_dataset_image_list_by_ID)

    for j in range(len(train_1f_camera_id_list)):

        print(train_1f_camera_id_list[j] + ' : ' + str(len(train_1f_dataset_image_list_by_ID[j])))

    
    display_image_6 = []
    display_image_4 = []
    for j in range(len(train_1f_dataset_image_list_by_ID[0])):

        for k in range(6):

            img = cv2.imread(train_1f_dataset_image_path + '/' + train_1f_dataset_image_list_by_ID[k][j], cv2.IMREAD_COLOR)
            resized_img = cv2.resize(img, None, fx=0.15, fy=0.15, interpolation=cv2.INTER_AREA)

            display_image_6.append(resized_img)

            img_h_6 = cv2.hconcat(display_image_6)

        for k in range(4):
            
            img = cv2.imread(train_1f_dataset_image_path + '/' + train_1f_dataset_image_list_by_ID[k+6][j], cv2.IMREAD_COLOR)
            resized_img = cv2.resize(img, None, fx=0.15, fy=0.15, interpolation=cv2.INTER_AREA)

            display_image_4.append(resized_img)

            img_h_4 = cv2.hconcat(display_image_4)

        cv2.imshow('Full Image from 6 Baisler CAMS', img_h_6)
        cv2.imshow('Full Image from 4 Galaxy CAMS', img_h_4)

        cv2.waitKey(0)

        cv2.destroyAllWindows()

        display_image_6.clear()
        display_image_4.clear()
    

    # In case that next dataset package is created by different camera IDs, clear the list of camera IDs
    train_1f_camera_id_list.clear()

    # Clear train image dataset for next dataset package
    train_1f_dataset_image_list_by_ID.clear()

    print('-----------------------------------------------------------------------------')