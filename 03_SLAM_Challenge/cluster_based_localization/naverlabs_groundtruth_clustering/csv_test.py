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

from sklearn.model_selection import train_test_split

from dataset_split import dataset_splitter

dataset_splitter().split(['./dataset_1F.csv', './dataset_B1.csv'], test_ratio=0.3)