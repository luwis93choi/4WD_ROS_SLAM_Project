### Image retrieval using CNN-based encoder (Feature extractor) ###
# Import the necessary packages
import tensorflow as tf
from tensorflow.keras.models import Model
from tensorflow.keras.models import load_model
from tensorflow.keras.datasets import mnist
from imutils import build_montages
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

def feature_matcher(queryImg, candidateImg):

    high_match_Img = []

    orb = cv2.ORB_create()

    brute_force_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    queryImg_cv = cv2.resize(queryImg,(300, 300))

    queryImg_keypoints = orb.detect(queryImg_cv, None)
    queryImg_keypoints, queryImg_descriptors = orb.compute(queryImg_cv, queryImg_keypoints)

    candidate_num = 0

    if queryImg_descriptors is not None:

        for img in candidateImg:

            try:

                img_cv = cv2.resize(img, (300, 300))

                candidateImg_keypoints = orb.detect(img_cv, None)
                candidateImg_keypoints, candidateImg_descriptors = orb.compute(img_cv, candidateImg_keypoints)

                if candidateImg_descriptors is not None:

                    matches = brute_force_matcher.match(queryImg_descriptors, candidateImg_descriptors)

                    #matches = sorted(matches, key=lambda x:x.distance)

                    match_ratio = float(len(matches)) / float(len(queryImg_descriptors))


                    if match_ratio >= 0.5:

                        print('Candidate Img[' + str(candidate_num) +'] --- Match Ratio : ' + '{:.2f}'.format(match_ratio) + '/ Matches : ' + str(len(matches)) + ' / candidateImg_descriptors : ' + str(len(candidateImg_descriptors)))

                        high_match_Img.append(img)
                
                elif candidateImg_descriptors is None:
                    # Exception : Case of featureless image
                    pass
                    #print('Candidate Img[' + str(candidate_num) +'] is featureless')

                candidate_num += 1
                
            finally:

                pass

    elif queryImg_descriptors is None:
        # Exception : Case of featureless image
        pass
        
    print('----------------------------------------------------------------')

    return high_match_Img

# Construct the argument parser
ap = argparse.ArgumentParser()
ap.add_argument('-d', '--dataset', type=str, required=True,
    help='Path to the dataset pickle file')
ap.add_argument('-m', '--model', type=str, required=True,
    help='Path to trained autoencoder')
ap.add_argument('-i', '--index', type=str, required=True,
    help='Path to features index file')
ap.add_argument('-s', '--sample', type=int, default=20,
    help='Number of testing queries to perform')
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

# Load the autoencoder model and index from disk
print('[INFO] Loading autoencoder and index...')
autoencoder = load_model(args['model'])
index = pickle.loads(open(args['index'], 'rb').read())

# Create the encoder model which consists of 'just' the encoder portion of the autoencoder
encoder = Model(inputs=autoencoder.input, outputs=autoencoder.get_layer('encoder').output)

# Quantify the contents of input testing images using the encoder
print('[INFO] Encoding testing images...')
features = encoder.predict(test_dataset)

### Take a random sample of images and turn them into queries
# Randomly sample a set of testing query image indexes
queryIdxs = list(range(0, test_dataset.shape[0]))
queryIdxs = np.random.choice(queryIdxs, size=args['sample'], replace=False)

# Loop over the testing indexes
for i in queryIdxs:
    # Take the features for the current query image
    # Find all similar images in our dataset and initialize the list of result images
    queryFeatures = features[i]
    results = perform_search(queryFeatures, index, maxResults=100)
    images = []

    # Loop over the results
    for (d, j) in results:
        # Grab the result image and convert it back to the range [0, 255]
        # Update the image list
        image = (train_dataset[j] * 255).astype('uint8')
        image = np.dstack([image] * 3)
        images.append(image)


    # Display the query image
    query = (test_dataset[i] * 255).astype('uint8')

    # Conduct feature matching and pick the images with over 90% match ratio
    high_match_results = feature_matcher(query, images)

    cv2.imshow('Query', query)

    # Build a montage from the results and display it
    montage = build_montages(high_match_results, (300, 300), (5, 2))[0]
    cv2.imshow('Results', montage)
    cv2.waitKey(0)

