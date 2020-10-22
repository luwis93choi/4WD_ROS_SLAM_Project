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

from matplotlib import pyplot as plt

from datasetLoader import indoorDataset

import csv

import classifier_nn

from torchsummary import summary

import datetime
import os
import sys

from dataset_split import dataset_splitter
'''
full_dataset_path_list = [
    './dataset_1F.csv',
    './dataset_B1.csv'
]
'''
full_dataset_path_list = [
    './dataset_B1.csv'
]
'''
nn_type = ['1F', 'B1']
'''
nn_type = ['B1']
type_index = 0

### Split dataset
datasets = dataset_splitter().split(full_dataset_path_list, test_ratio=0.3)

print(datasets)

### Run through all the datasets (1F and B1)
for datasetpath in datasets:

    ### Prepare Dataloader
    BATCH_SIZE = 32
    EPOCH = 200
    resolution = [224, 224]

    transform = transforms.Compose([
        transforms.RandomHorizontalFlip(p=0.5),
        transforms.RandomPerspective(distortion_scale=0.5, p=0.5),
        transforms.Resize(resolution),
        transforms.ToTensor(),
        transforms.Normalize((0.5,0.5,0.5),(0.5,0.5,0.5)),
    ])

    # Prepare the dataloader for training and apply Data Augmentation
    train_dataset = indoorDataset(datasetpath=datasetpath['train'], 
                                  x_name='ImgName', 
                                  y_name='Cluster_Label',
                                  resolution=resolution,
                                  transform=transform)

    train_loader = DataLoader(dataset=train_dataset,
                              batch_size=BATCH_SIZE,
                              shuffle=True,
                              num_workers=0)

    # Prepare the dataloader for validation
    valid_dataset = indoorDataset(datasetpath=datasetpath['valid'], 
                                  x_name='ImgName', 
                                  y_name='Cluster_Label',
                                  resolution=resolution,
                                  transform=transform)

    valid_loader = DataLoader(dataset=valid_dataset,
                              batch_size=BATCH_SIZE,
                              shuffle=True,
                              num_workers=0)

    ### Prepare Neural Network
    cluster_labels = []
    with open('./dataset_B1.csv', 'r', encoding='utf-8') as dataset_file:

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

    # Train over multiple times
    loss_val = []

    start_time = str(datetime.datetime.now())

    for epoch in range(EPOCH):
        
        running_loss = 0.0

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
            print('[%d, %5d / %d] loss: %.3f' %
                  (epoch + 1, sequence_num, train_loader.__len__(), running_loss))
            
            loss_val.append(running_loss)
            
            running_loss = 0.0

            sequence_num += 1
        
        epoch_complete_time = str(datetime.datetime.now())

        torch.save(vgg19, './vgg19_' + nn_type[type_index] + '_' + epoch_complete_time + '.pth')

        # Plot and save the training results
        plt.cla()
        plt.plot(range(len(loss_val)), loss_val)
        plt.title('Traning Results - Loss Value')
        plt.xlabel('Training Batch Num')
        plt.ylabel('Loss')
        plt.savefig('./Training Results ' + nn_type[type_index] + '_'  + epoch_complete_time + '.png')

    type_index += 1

    del vgg19