# Set the matplotlib backend so figures can be saved in the background
import matplotlib
matplotlib.use('Agg')

# Import the necessary packages
import tensorflow as tf
from convautoencoder.autoencoder import ConvAutoencoder
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.datasets import mnist
import matplotlib.pyplot as plt
import numpy as np
import argparse
import pickle
import cv2

### GPU Support Activation ###
# Enable the use of GPUs by allowing their memory growth
# Reference : https://inpages.tistory.com/155
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
  try:
    # Currently, memory growth needs to be the same across GPUs
    for gpu in gpus:
      tf.config.experimental.set_memory_growth(gpu, True)
    logical_gpus = tf.config.experimental.list_logical_devices('GPU')
    print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
  except RuntimeError as e:
    # Memory growth must be set before GPUs have been initialized
    print(e)

# Construct the argument parser to receive the path to the dataset
ap = argparse.ArgumentParser()
ap.add_argument('-d', '--dataset', type=str, required=True,
    help='Path to the dataset pickle file')
ap.add_argument('-m', '--model', type=str, required=True, 
    help='Path to output trained autoencoder')
ap.add_argument('-p', '--plot', type=str, default='plot.png',
    help='Path to output plot file')
ap.add_argument('-v', '--vis', type=str, default='recon_vis.png',
    help='Path to output reconstruction visualization file')
args = vars(ap.parse_args())

# Open dataset pickle file
train_dataset = []
with open(args['dataset'], 'rb') as f:
    dataset_dict = pickle.load(f)
    train_dataset = dataset_dict["gray_mat_train"]
    test_dataset = dataset_dict["gray_mat_test"]
  
print(dataset_dict.keys())

train_dataset = np.expand_dims(train_dataset, axis=-1)
test_dataset = np.expand_dims(test_dataset, axis=-1)

train_dataset = train_dataset.astype('float32') / 255.0
test_dataset = test_dataset.astype('float32') / 255.0

### Training Configurations ###
EPOCHS = 20
INIT_LearningRate = 1e-3
BatchSize = 30

autoencoder = ConvAutoencoder.build(300, 300, 1)

opt = Adam(lr=INIT_LearningRate, decay=INIT_LearningRate/EPOCHS)

autoencoder.compile(loss='mse', optimizer=opt)

H = autoencoder.fit(
    train_dataset, train_dataset,
    validation_data=(test_dataset, test_dataset),
    epochs=EPOCHS,
    batch_size=BatchSize
)

# Construct a plot that plots and saves the training history
N = np.arange(0, EPOCHS)
plt.style.use("ggplot")
plt.figure()
plt.plot(N, H.history["loss"], label="train_loss")
plt.plot(N, H.history["val_loss"], label="val_loss")
plt.title("Training Loss and Accuracy")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend(loc="lower left")
plt.savefig(args["plot"])

# Serialize the autoencoder model to disk
print("[INFO] saving autoencoder...")
autoencoder.save(args["model"], save_format="h5")

def visualize_predictions(decoded, ground_truth_img, samples=10):
    # Initialize the list of output images
    outputs = None
    # Loop over the number of output samples
    for i in range(0, samples):
        # Grab the original image and reconstructed image
        original = (ground_truth_img[i] * 255).astype('uint8')
        recon = (decoded[i] * 255).astype('uint8')

        # Stack the original image and reconstructed image side-by-side
        output = np.hstack([original, recon])

        # If the output array is empty, initialize it as the current side-by-side image display
        if outputs is None:
            outputs = output

        # Otherwise, vertically stack the outputs
        else:
            outputs = np.vstack([outputs, output])

    # Return the output image
    return outputs

### Prediction ###
# Use the convolutional autencoder to make predictions on the testing images
# Construct the visualization and save the results to the disk
print('[INFO] Making predictions...')
decoded = autoencoder.predict(test_dataset)
vis = visualize_predictions(decoded, test_dataset)
cv2.imwrite(args['vis'], vis)