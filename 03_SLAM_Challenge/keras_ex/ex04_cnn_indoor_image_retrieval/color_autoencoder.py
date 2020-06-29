# Reference 1 : https://stackabuse.com/autoencoders-for-image-reconstruction-in-python-and-keras/

import matplotlib.pyplot as plt
import numpy as np
import argparse
import pickle
import cv2

import tensorflow as tf
from tensorflow.keras.layers import Dense, Flatten, Reshape, Input, InputLayer
from tensorflow.keras.models import Sequential, Model
from tensorflow.keras.models import load_model 
from imutils import build_montages

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

# Construct the argument parser
ap = argparse.ArgumentParser()
ap.add_argument('-d', '--dataset', type=str, required=True,
    help='Path to the dataset pickle file')
ap.add_argument('-i', '--index', type=str, required=True,
    help='Path to output features index file')
ap.add_argument('-s', '--sample', type=int, default=10,
    help='Number of testing queries to perform')
args = vars(ap.parse_args())

# Open dataset pickle file
with open(args['dataset'], 'rb') as f:
    dataset_dict = pickle.load(f)
    train_dataset = dataset_dict["gray_mat_train"]
    test_dataset = dataset_dict["gray_mat_test"]

train_dataset = np.array(train_dataset, dtype=np.float32) / 255.0
test_dataset = np.array(test_dataset, dtype=np.float32) / 255.0

#plt.imshow(train_dataset[0])
#plt.show()

def build_color_CONVautoencoder(img_shape, code_size):
    # Encoder
    encoder = Sequential()
    encoder.add(InputLayer(img_shape))
    encoder.add(Flatten())
    encoder.add(Dense(code_size))
  
    # Decoder
    decoder = Sequential()
    decoder.add(InputLayer((code_size, )))
    decoder.add(Dense(np.prod(img_shape)))
    decoder.add(Reshape(img_shape))

    return encoder, decoder

IMG_SHAPE = train_dataset.shape[1:]
encoder, decoder = build_color_CONVautoencoder(IMG_SHAPE, 32)

inp = Input(IMG_SHAPE)
code = encoder(inp)
reconstruction = decoder(code)

autoencoder = Model(inp, reconstruction)
autoencoder.compile(optimizer='adamax', loss='mse')

print(autoencoder.summary())

history = autoencoder.fit(
    train_dataset, train_dataset,
    validation_data=(test_dataset, test_dataset),
    epochs=20,
    batch_size=20
)

plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()

# Construct a dictionary that maps the index of the MNIST training images to
# its corresponding latent-space representations

#for idx in range(len(autoencoder.layers)):
#    print(autoencoder.get_layer(index=idx).name)

#CONVencoder = Model(inputs=autoencoder.get_layer('input_3').input, outputs=autoencoder.get_layer('sequential').get_layer('dense').output)

#features = CONVencoder.predict(train_dataset)

indexes = list(range(0, train_dataset.shape[0]))
data = {'indexes':indexes, 'features':features}

# Write the data dictionary to disk
print('[INFO] Saving index...')
f = open(args['index'], 'wb')
f.write(pickle.dumps(data))
f.close()

# Function for computing the similarity between two feature vectors using Euclidean distance
def euclidean(a, b):
    # Compute and return the euclidean distance between two vectors
    return np.linalg.norm(a - b)

# Searching Function
def perform_search(queryFeatures, index, maxResults=64):
    # Initialize the list of results
    results = []

    # Loop over the indexes
    for i in range(0, len(index['features'])):
        # Compute the euclidean distance between the query image features 
        # and the current input image features
        # Update the result list with 2-tuple consisting of the computed distance
        # and the index of the image
        d = euclidean(queryFeatures, index['features'][i])
        results.append((d, i))

    # Sort the results and grab the top results
    results = sorted(results)[:maxResults]

    # Return the list of results
    return results

# Load the autoencoder model and index from disk
print('[INFO] Loading autoencoder and index...')
index = pickle.loads(open(args['index'], 'rb').read())

# Quantify the contents of input testing images using the encoder
print('[INFO] Encoding testing images...')
test_features = CONVencoder.predict(test_dataset)

### Take a random sample of images and turn them into queries
# Randomly sample a set of testing query image indexes
queryIdxs = list(range(0, test_dataset.shape[0]))
queryIdxs = np.random.choice(queryIdxs, size=args['sample'], replace=False)

# Loop over the testing indexes
for i in queryIdxs:
    # Take the features for the current query image
    # Find all similar images in our dataset and initialize the list of result images
    queryFeatures = test_features[i]
    results = perform_search(queryFeatures, index, maxResults=255)

    # Display the image with the closest euclidean distance
    cv2.imshow('Query', train_dataset[i]) 
    for (d, j) in results:
        cv2.imshow('Results', train_dataset[j])   
        cv2.waitKey(0)