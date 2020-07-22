# VGG 19 PyTorch Customization Reference : https://minjoos.tistory.com/6

from __future__ import print_function

import torch
import torchvision
import torchvision.transforms as transforms

import torch.nn as nn
import torch.nn.functional as F

import torch.optim as optim

from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader

from datasetLoader import indoorDataset

import csv

import classifier_nn

from torchsummary import summary

import datetime
import os
import sys

### Prepare Dataloader
batch_size = 16
resolution = [224, 224]

dataset = indoorDataset(datasetpath='./dataset_1F.csv', 
                        x_name='ImgName', 
                        y_name='Cluster_Label',
                        resolution=resolution)

train_loader = DataLoader(dataset=dataset,
                          batch_size=batch_size,
                          shuffle=True,
                          num_workers=0)

### Prepare Neural Network
cluster_labels = []
with open('././dataset_1F.csv', 'r', encoding='utf-8') as dataset_file:

    reader = csv.DictReader(dataset_file)

    for dataDict in reader:

        cluster_labels.append(int(dataDict['Cluster_Label']))

label_num = max(cluster_labels) + 1
print('Number of labels for classification : ' + str(label_num))

# Load main processing unit for neural network
PROCESSOR = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

# Create clustering-based localization neural network object
vgg19 = classifier_nn.Net(label_num)

# Bind Neural Network to Processing Unit (GPU if possible)
vgg19.to(PROCESSOR)

param = list(vgg19.parameters())

# Display neural network structure using torchsummary
summary(vgg19, (3, 224, 224))

# Declare training criterion and optimizer
criterion = nn.CrossEntropyLoss().cuda()
optimizer = optim.Adam(vgg19.parameters(), lr=0.00001)

### Neural Network Training

# Set neural network in training mode
vgg19.train()
sequence_num = 1
for input_img, label in train_loader:

    # Prepare input data and output label as pytorch tensor
    # Bind input data and output label to processing unit, so that neural network and data can be loaded on same processing unit
    input_img = Variable(input_img.to(PROCESSOR))
    label = Variable(label.to(PROCESSOR))

    # Supply data into the network for training
    output = vgg19(input_img)

    print('Training Sequence : ' + str(sequence_num) + '/' + str(train_loader.__len__()), end='\r')
    sys.stdout.flush()

    sequence_num += 1

torch.save(vgg19, './vgg19_' + str(datetime.datetime.now()))