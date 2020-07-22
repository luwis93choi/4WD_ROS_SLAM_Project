import torch
import numpy as np
from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader

from PIL import Image
import torchvision.transforms.functional as TF

import csv

import cv2

class indoorDataset(Dataset):

    def __init__(self, datasetpath, x_name, y_name, resolution=[300, 300]):
        
        self.len = 0
        self.x_data = []
        self.y_data = []

        self.input_resolution = resolution

        with open(datasetpath, 'r', encoding='utf-8') as dataset_file:

            reader = csv.DictReader(dataset_file)

            for dataDict in reader:

                self.len += 1
                self.x_data.append(dataDict[x_name])
                self.y_data.append(int(dataDict[y_name]))

    def __getitem__(self, index):

        image = Image.open(self.x_data[index])
        resized_img = image.resize((self.input_resolution[0], self.input_resolution[1]))

        input_tensor_img = TF.to_tensor(resized_img)

        label_tensor = torch.tensor(self.y_data[index])

        return input_tensor_img, label_tensor

    def __len__(self):
        return self.len

