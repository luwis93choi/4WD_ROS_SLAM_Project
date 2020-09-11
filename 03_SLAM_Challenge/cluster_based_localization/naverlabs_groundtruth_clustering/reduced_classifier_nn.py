# VGG 19 PyTorch Customization Reference : https://minjoos.tistory.com/6

import torch
import torchvision
import torchvision.transforms as transforms

import torch.nn as nn
import torch.nn.functional as F

import matplotlib.pyplot as plt
import numpy as np

class Net(nn.Module):

    def __init__(self, output_label_num):
        super(Net, self).__init__()

        # Convolutional Layers --- Feature Extraction
        self.conv = nn.Sequential(

            #3 224 128
            nn.Conv2d(3, 64, 3, padding=1), nn.LeakyReLU(0.2),
            nn.BatchNorm2d(64),
            nn.Conv2d(64, 64, 3, padding=1), nn.LeakyReLU(0.2),
            nn.BatchNorm2d(64),
            nn.MaxPool2d(2, 2),

            #64 112 64
            nn.Conv2d(64, 128, 3, padding=1), nn.LeakyReLU(0.2),
            nn.BatchNorm2d(128),
            nn.Conv2d(128, 128, 3, padding=1), nn.LeakyReLU(0.2),
            nn.BatchNorm2d(128),
            nn.MaxPool2d(2, 2)
        )

        # Final Pooling 
        #512 7 4
        self.avg_pool = nn.AvgPool2d(7)

        # Classification Layer
        #512 1 1
        self.classifier = nn.Linear(8192, output_label_num)

    def forward(self, x):

        # Convolutional Layers --- Feature Extraction
        features = self.conv(x)

        # Final Pooling
        x = self.avg_pool(features)

        x = x.view(features.size(0), -1)

        # Classification Layer
        x = self.classifier(x)

        return x