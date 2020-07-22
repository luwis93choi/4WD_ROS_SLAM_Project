# scikit-learn Clustering Libraries
from sklearn.cluster import AgglomerativeClustering

# matplotlib Plotting Libraries
from matplotlib import pyplot as plt
from matplotlib import patches as mpatches

# Numpy for array handling
import numpy as np

# h5py for reading groundtruth data
import os
import sys
import h5py
import json

# Custom class for cluster plotting
from plot_clustering.plot_clustering import cluster_plotter

# csv for dataset merging with cluster labels
import csv

############################################ Groundtruth Compilation & Clustering #######################################################

### Compiling groundtruth files
groundtruth_path_1F = [
    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset/1f/train/2019-04-16_14-35-00/groundtruth.hdf5',
    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset/1f/train/2019-08-20_11-32-05/groundtruth.hdf5'
]

groundtruth_path_B1 = [
    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset/b1/train/2019-04-16_15-35-46/groundtruth.hdf5',
    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset/b1/train/2019-04-16_16-14-48/groundtruth.hdf5',
    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset/b1/train/2019-08-20_10-41-18/groundtruth.hdf5'
]

groundtruth_1F = []
for i in range(len(groundtruth_path_1F)):
    groundtruth_1F.append(h5py.File(groundtruth_path_1F[i], 'r'))

groundtruth_B1 = []
for i in range(len(groundtruth_path_B1)):
    groundtruth_B1.append(h5py.File(groundtruth_path_B1[i], 'r'))

### 1F Dataset Clustering
### Collect all the XYZ coordinates from groundtruth dataset
groundtruth_1F_total_pose = []
groundtruth_1F_total_quaternion = []
for i in range(len(groundtruth_1F)):
    print('----- Collecting Groundtruth for 1F : Dataset[' + str(i) +'] -----')

    key_list = list(groundtruth_1F[i].keys())

    CAM_index = 0
    for key_val in key_list:

        groundtruth_1F_pose = []
        groundtruth_1F_quaternion = []
        
        if key_val.find('_pose') is not -1:
            if key_val.find('lidar') is -1:
                print('Accessing : ' + key_val)

                for pose_info in groundtruth_1F[i][key_val].value:

                    # Collect only X, Y, Z coordinates of dataset
                    groundtruth_1F_pose.append(pose_info[0:3])

                    # Collect only quaternion of dataset
                    groundtruth_1F_quaternion.append(pose_info[3:7])

                #print(groundtruth_1F_pose)
                groundtruth_1F_total_pose.extend(groundtruth_1F_pose)
                groundtruth_1F_total_quaternion.extend(groundtruth_1F_quaternion)

                '''
                ### Clustering XYZ coordinates of groundtruth data by each CAM using Agglomerative Clustering
                clustering = AgglomerativeClustering(n_clusters=None, linkage='ward', distance_threshold=5.0).fit(groundtruth_1F_pose)

                print(clustering.labels_)

                max_label = int(max(clustering.labels_))
                sample_num = [0] * (max_label + 1)
                for k in clustering.labels_:
                    sample_num[int(k)] += 1
                total_val = 0
                for k in sample_num:
                    total_val = total_val + k

                print(sample_num)
                print(total_val)
                
                plotter = cluster_plotter()

                plotter.draw(groundtruth_1F_pose, clustering.labels_, '1F Clustering CAM ' + str(CAM_index) + ' / 5cm')

                del clustering
                CAM_index += 1
                '''

### Clustering XYZ coordinates of entire groundtruth data using Agglomerative Clustering
print('[INFO] Clustering 1F pose data...')
clustering_1F = AgglomerativeClustering(n_clusters=None, linkage='ward', distance_threshold=0.25).fit(groundtruth_1F_total_pose)
print('[INFO] 1F Clustering completed')

print(clustering_1F.labels_)

plotter = cluster_plotter()

#plotter.draw(groundtruth_1F_total_pose, clustering_1F.labels_, '1F Clustering / All CAMs / 25cm')

### B1 Dataset Clustering
### Collect all the XYZ coordinates from groundtruth dataset
groundtruth_B1_total_pose = []
groundtruth_B1_total_quaternion = []
for i in range(len(groundtruth_B1)):
    print('----- Collecting Groundtruth for B1 : Dataset[' + str(i) +'] -----')

    key_list = list(groundtruth_B1[i].keys())

    CAM_index = 0
    for key_val in key_list:

        groundtruth_B1_pose = []
        groundtruth_B1_quaternion =[]

        if key_val.find('_pose') is not -1:
            if key_val.find('lidar') is -1:
                print('Accessing : ' + key_val)

                for pose_info in groundtruth_B1[i][key_val].value:

                    # Collect only X, Y, Z coordinates of dataset
                    groundtruth_B1_pose.append(pose_info[0:3])

                    # Collect only quaternion of dataset
                    groundtruth_B1_quaternion.append(pose_info[3:7])

                #print(groundtruth_B1_pose)
                groundtruth_B1_total_pose.extend(groundtruth_B1_pose)
                groundtruth_B1_total_quaternion.extend(groundtruth_B1_quaternion)

                '''
                ### Clustering XYZ coordinates of groundtruth data by each CAM using Agglomerative Clustering
                clustering = AgglomerativeClustering(n_clusters=None, linkage='ward', distance_threshold=5.0).fit(groundtruth_B1_pose)

                print(clustering.labels_)

                max_label = int(max(clustering.labels_))
                sample_num = [0] * (max_label + 1)
                for k in clustering.labels_:
                    sample_num[int(k)] += 1
                total_val = 0
                for k in sample_num:
                    total_val = total_val + k

                print(sample_num)
                print(total_val)
                
                plotter = cluster_plotter()

                plotter.draw(groundtruth_1F_pose, clustering.labels_, '1F Clustering CAM ' + str(CAM_index) + ' / 5cm')

                del clustering
                CAM_index += 1
                '''

### Clustering XYZ coordinates of entire groundtruth data using Agglomerative Clustering
print('[INFO] Clustering B1 pose data...')
clustering_B1 = AgglomerativeClustering(n_clusters=None, linkage='ward', distance_threshold=0.25).fit(groundtruth_B1_total_pose)
print('[INFO] B1 Clustering completed')

print(clustering_B1.labels_)

plotter = cluster_plotter()

#plotter.draw(groundtruth_B1_total_pose, clustering_B1.labels_, 'B1 Clustering / All CAMs / 25cm')

############################################ Merging Image Name, Cluster Label, XYZ, Quaternion #######################################################
image_folder_1F = [
    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset/1f/train/2019-04-16_14-35-00/images',
    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset/1f/train/2019-08-20_11-32-05/images'
]

image_folder_B1 = [
    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset/b1/train/2019-04-16_15-35-46/images',
    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset/b1/train/2019-04-16_16-14-48/images',
    '/media/luwis/Linux Workspace/ICSL_Project/Visual SLAM/indoor_dataset/dataset/b1/train/2019-08-20_10-41-18/images'
]

### Image Name Compiling
ImgName_1F = []
for folderPath in image_folder_1F:
    ImgName_1F.extend([folderPath + '/' + filename for filename in sorted(os.listdir(folderPath))])
    #ImgName_1F.extend(sorted(os.listdir(folderPath)))

ImgName_B1 = []
for folderPath in image_folder_B1:
    ImgName_B1.extend([folderPath + '/' + filename for filename in sorted(os.listdir(folderPath))])
    #ImgName_B1.extend(sorted(os.listdir(folderPath)))

### All Pose Data is compiled at 'groundtruth_1F_total_pose' and 'groundtruth_B1_total_pose'

### All Quaternion Data is compiled at 'groundtruth_1F_total_quaternion' and 'groundtruth_B1_total_quaternion'

### Merging 1F dataset

print('----- Merging 1F Image Name, Cluster Label, XYZ, Quaternion -----')
print('Number of ImgName : ' + str(len(ImgName_1F)))
print('Number of Cluster Labels : ' + str(len(clustering_1F.labels_)))
print('Number of Pose : ' + str(len(groundtruth_1F_total_pose)))
print('Number of Quaternion : ' + str(len(groundtruth_1F_total_quaternion)))

fieldnames = ['ImgName', 'Cluster_Label', 'Coordinate', 'Quaternion']

with open('./dataset_1F.csv', 'w', encoding='utf-8') as dataset_file:

    writer = csv.DictWriter(dataset_file, fieldnames=fieldnames)

    writer.writeheader()

    for i in range(len(ImgName_1F)):

        writer.writerow({
            'ImgName' : ImgName_1F[i],
            'Cluster_Label' : clustering_1F.labels_[i],
            'Coordinate' : groundtruth_1F_total_pose[i],
            'Quaternion' : groundtruth_1F_total_quaternion[i]
        })

### Merging B1 dataset

print('----- Merging B1 Image Name, Cluster Label, XYZ, Quaternion -----')
print('Number of ImgName : ' + str(len(ImgName_B1)))
print('Number of Cluster Labels : ' + str(len(clustering_B1.labels_)))
print('Number of Pose : ' + str(len(groundtruth_B1_total_pose)))
print('Number of Quaternion : ' + str(len(groundtruth_B1_total_quaternion)))

with open('./dataset_B1.csv', 'w', encoding='utf-8') as dataset_file:

    writer = csv.DictWriter(dataset_file, fieldnames=fieldnames)

    writer.writeheader()

    for i in range(len(ImgName_B1)):

        writer.writerow({
            'ImgName' : ImgName_B1[i],
            'Cluster_Label' : clustering_B1.labels_[i],
            'Coordinate' : groundtruth_B1_total_pose[i],
            'Quaternion' : groundtruth_B1_total_quaternion[i]
        })

### Cluster Distribution Plotting
dist_1F = [0] * (int(max(clustering_1F.labels_)) + 1)
dist_x = range(max(clustering_1F.labels_) + 1)
for cluster_label in clustering_1F.labels_:
    dist_1F[cluster_label] += 1

#plt.bar(dist_x, dist_1F)
plt.hist(clustering_1F.labels_, bins=int(max(clustering_1F.labels_)) + 1)
plt.title('1F Cluster Distribution')
plt.xlabel('Cluster Label')
plt.ylabel('Number of Images in Each Cluster')
plt.show()

dist_B1 = [0] * (int(max(clustering_B1.labels_)) + 1)
dist_x = range(max(clustering_B1.labels_) + 1)
for cluster_label in clustering_B1.labels_:
    dist_B1[cluster_label] += 1

#plt.bar(dist_x, dist_B1)
plt.hist(clustering_B1.labels_, bins=int(max(clustering_B1.labels_)) + 1)
plt.title('B1 Cluster Distribution')
plt.xlabel('Cluster Label')
plt.ylabel('Number of Images in Each Cluster')
plt.show()

