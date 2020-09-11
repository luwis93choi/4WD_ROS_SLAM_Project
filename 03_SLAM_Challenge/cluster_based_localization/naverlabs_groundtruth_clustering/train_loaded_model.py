from __future__ import print_function

import torch
import torchvision
import torchvision.transforms as transforms

import torch.nn as nn
import torch.nn.functional as F

import torch.optim as optim

from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader

from matplotlib import pyplot as plt

from datasetLoader import indoorDataset

import csv

import classifier_nn
import reduced_classifier_nn

from torchsummary import summary

import datetime
import os
import sys

from dataset_split import dataset_splitter

nn_type = ['1F', 'B1']
type_index = 0

### Prepare Dataloader
BATCH_SIZE = 32
EPOCH = 100
resolution = [224, 224]

transform = transforms.Compose([
    transforms.RandomHorizontalFlip(p=0.5),
    transforms.RandomPerspective(distortion_scale=0.5, p=0.5),
    transforms.Resize(resolution),
    transforms.ToTensor(),
    transforms.Normalize((0.5,0.5,0.5),(0.5,0.5,0.5)),
])

# Prepare the dataloader for training and apply Data Augmentation
train_dataset = indoorDataset(datasetpath='./train_dataset_1F.csv', 
                              x_name='ImgName', 
                              y_name='Cluster_Label',
                              resolution=resolution,
                              transform=transform)

train_loader = DataLoader(dataset=train_dataset,
                          batch_size=BATCH_SIZE,
                          shuffle=True,
                          num_workers=0)


### Prepare Neural Network
cluster_labels = []
with open('./dataset_1F.csv', 'r', encoding='utf-8') as dataset_file:

    reader = csv.DictReader(dataset_file)

    for dataDict in reader:

        cluster_labels.append(int(dataDict['Cluster_Label']))

label_num = max(cluster_labels) + 1
print('Number of labels for classification : ' + str(label_num))

# Load main processing unit for neural network
PROCESSOR = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

# Create clustering-based localization neural network object
vgg19 = torch.load('./vgg19_1F_2020-07-27 18:43:26.665873.pth')
#vgg19 = reduced_classifier_nn.Net(label_num)

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

# Train over multiple times
loss_val = []

start_time = str(datetime.datetime.now())

epoch = 85

break_sig = False

while True:
    
    running_loss = 0.0

    loss_eval_list = []
    avg_loss = 0.0

    sequence_num = 1
    for i, data in enumerate(train_loader, 0):

        #print('Training Sequence : ' + str(sequence_num) + '/' + str(train_loader.__len__()))
        #sys.stdout.flush()

        # Prepare input data and output label as pytorch tensor
        # Bind input data and output label to processing unit, so that neural network and data can be loaded on same processing unit
        input_img, label = data

        input_img = Variable(input_img.to(PROCESSOR))
        label = Variable(label.to(PROCESSOR))

        # Zero the parameter gradients
        optimizer.zero_grad()

        # Forward the network while supplying training data
        output = vgg19(input_img)

        loss = criterion(output, label)

        # Backpropagate the network
        loss.backward()

        # Optimize the network
        optimizer.step()

        # Print and save current statistics
        running_loss += loss.item()

        loss_val.append(running_loss)
        loss_eval_list.append(running_loss)
        avg_loss = sum(loss_eval_list) / len(loss_eval_list)

        print('[%d, %5d / %d] loss: %.3f / avg_loss : %.3f' %
                (epoch + 1, sequence_num, train_loader.__len__(), running_loss, avg_loss))
        
        if avg_loss < 0.001:
            break_sig = True
            break

        running_loss = 0.0

        sequence_num += 1
    
    epoch += 1

    epoch_complete_time = str(datetime.datetime.now())

    torch.save(vgg19, './vgg19_' + nn_type[type_index] + '_' + epoch_complete_time + '.pth')

    # Plot and save the training results
    plt.cla()
    plt.plot(range(len(loss_val)), loss_val)
    plt.title('Traning Results - Loss Value')
    plt.xlabel('Training Batch Num')
    plt.ylabel('Loss')
    plt.savefig('./Training Results ' + nn_type[type_index] + '_'  + epoch_complete_time + '.png')

    if break_sig is True:
        break

type_index += 1