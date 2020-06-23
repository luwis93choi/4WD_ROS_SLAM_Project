# Set the matplotlib backend so figures can be saved in the background
import matplotlib
matplotlib.use('Agg')

# Import the necessary packages
import tensorflow as tf
from autoencoder.convautoencoder import ConvAutoencoder
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.datasets import mnist
import matplotlib.pyplot as plt
import numpy as np
import argparse
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

# Construct the argument parser
ap = argparse.ArgumentParser()
ap.add_argument('-m', '--model', type=str, required=True, 
    help='Path to output trained autoencoder')
ap.add_argument('-v', '--vis', type=str, default='recon_vis.png',
    help='Path to output reconstruction visualization file')
ap.add_argument('-p', '--plot', type=str, default='plot.png',
    help='Path to output plot file')
args = vars(ap.parse_args())

### Training Configuration ###
# Initialize the number of epochs to train for, initial learning rate, and batch size
EPOCHS = 20
INIT_LR = 1e-3
BS = 32

# Load MNIST Dataset
print('[INFO] Loading MINIST dataset...')
((trainX, _), (testX, _)) = mnist.load_data()

# Add a channel dimension to every image in the dataset, then scale the pixel intensities to the range [0, 1]
trainX = np.expand_dims(trainX, axis=-1)
testX = np.expand_dims(testX, axis=-1)

trainX = trainX.astype('float32') / 255.0
testX = testX.astype('float32') / 255.0

# Construct the convolutional autoencoder
print('[INFO] Building autoencoder...')

autoencoder = ConvAutoencoder.build(28, 28, 1)

opt = Adam(lr=INIT_LR, decay=INIT_LR/EPOCHS)

autoencoder.compile(loss='mse', optimizer=opt)

# Train the convolutional autoencoder
H = autoencoder.fit(
    trainX, trainX,
    validation_data=(testX, testX),
    epochs=EPOCHS,
    batch_size=BS
)

### Prediction ###
# Use the convolutional autencoder to make predictions on the testing images
# Construct the visualization and save the results to the disk
print('[INFO] Making predictions...')
decoded = autoencoder.predict(testX)
vis = visualize_predictions(decoded, testX)
cv2.imwrite(args['vis'], vis)

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