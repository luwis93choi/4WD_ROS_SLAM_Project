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
import os

from sklearn.model_selection import train_test_split

from dataset_split import dataset_splitter

from PIL import Image
import torchvision.transforms.functional as TF

import classifier_nn

### Prepare Dataloader
BATCH_SIZE = 1
EPOCH = 3
resolution = [224, 224]

# Prepare the dataloader for validation
valid_dataset = indoorDataset(datasetpath='./train_dataset_1F.csv', 
                                x_name='ImgName', 
                                y_name='Cluster_Label',
                                resolution=resolution)

valid_loader = DataLoader(dataset=valid_dataset,
                            batch_size=BATCH_SIZE,
                            shuffle=False,
                            num_workers=0)

PROCESSOR = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

model = torch.load('./vgg19_1F_2020-07-28 12:27:24.041907.pth')

model.eval()

model.to(PROCESSOR)

summary(model, (3, 224, 224))

correct = 0
for i, data in enumerate(valid_loader, 0):

    input_img, label = data

    input_img = Variable(input_img.to(PROCESSOR))
    label = Variable(label.to(PROCESSOR))

    output = model(input_img)

    _, predicted = torch.max(output, 1)

    prediction = str(predicted.tolist()[0])
    answer_label = str(label.tolist()[0])

    if prediction is answer_label:
        result = 'Correct'
        correct += 1
    else:
        result = 'Wrong'

    print('[' + result + '] ' + 'Prediction : ' + prediction + ' / Answer : ' + answer_label)

print('Correction Ratio : ' + str(float(correct) / float(len(valid_loader))))