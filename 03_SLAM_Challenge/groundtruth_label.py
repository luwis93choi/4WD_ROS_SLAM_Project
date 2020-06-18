import os
import sys

import h5py

import json

groundtruth_path = '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset_light/1f/groundtruth.hdf5'

floor = '1f'
dataset_path = '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset_light/1f/train'

# Train image file name Read
train_image_file_list = sorted(os.listdir(dataset_path))

for i in range(1022):
    print('./train/' + str(train_image_file_list[i]))

# groundtruth file H5DF Read
groundtruth_file = h5py.File(groundtruth_path, 'r')

sensor_key_list = list(groundtruth_file.keys())

target_index = 0

target_group = sensor_key_list[target_index]

#print(target_group)

#print(groundtruth_file[target_group].value)

file_index = 0
end_index = 1021

groundtruth_label_file = open('groundtruth_label', 'w')
groundtruth_label_file.write('[' + '\n')
groundtruth_label_file.close()

#print(len(groundtruth_file[target_group].value) - 1)

for pose_info in groundtruth_file[target_group].value:

    name = train_image_file_list[file_index]

    tx = pose_info[0]   # True X
    ty = pose_info[1]   # True Y
    tz = pose_info[2]   # True Z
    qw = pose_info[3]   # Quaternion W
    qx = pose_info[4]   # Quaternion X
    qy = pose_info[5]   # Quaternion Y
    qz = pose_info[6]   # Quaternion Z

    groundtruth_JSON = {

        #"floor" : floor,
        #"name" : name,
        "qw" : str(qw),
        "qx" : str(qx),
        "qy" : str(qy),
        "qz" : str(qz),
        "x" : str(tx),
        "y" : str(ty),
        "z" : str(tz)
    }

    # print(groundtruth_JSON)

    groundtruth_label_file = open('groundtruth_label', 'a')

    if file_index == end_index:
        groundtruth_label_file.write(json.dumps(groundtruth_JSON, indent=4, sort_keys=True) + '\n')
        break
    else:
        groundtruth_label_file.write(json.dumps(groundtruth_JSON, indent=4, sort_keys=True) + ',' + '\n')

    groundtruth_label_file.close()

    file_index += 1

groundtruth_label_file = open('groundtruth_label', 'a')
groundtruth_label_file.write('\n' + ']')
groundtruth_file.close()