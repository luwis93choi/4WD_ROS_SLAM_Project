from datasetLoader import indoorDataset

import torch
import torchvision
import torchvision.transforms as transforms

import torch.nn as nn
import torch.nn.functional as F

import torch.optim as optim

from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader

from torchsummary import summary

import matplotlib.pyplot as plt
import numpy as np
import csv

batch_size = 32
resolution = [224, 224]

dataset = indoorDataset(datasetpath='./dataset_1F.csv', 
                        x_name='ImgName', 
                        y_name='Cluster_Label',
                        resolution=resolution)

train_loader = DataLoader(dataset=dataset,
                          batch_size=batch_size,
                          shuffle=True,
                          num_workers=0)

device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

vgg16_model = torchvision.models.vgg16(pretrained=True)

vgg16_model.to(device)
summary(vgg16_model, (3, 224, 224))

vgg16_model.train()
sequence_num = 1
for input_img, label in train_loader:

    input_img = Variable(input_img.to(device))
    label = Variable(label.to(device))
    output = vgg16_model(input_img)

    print('Training Sequence : ' + str(sequence_num) + '/' + str(train_loader.__len__()))

    sequence_num += 1