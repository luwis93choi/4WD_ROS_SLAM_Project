# Reference 1 : https://www.pyimagesearch.com/2020/03/30/autoencoders-for-content-based-image-retrieval-with-keras-and-tensorflow/

# import the necessary packages
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.layers import BatchNormalization
from tensorflow.keras.layers import Conv2D
from tensorflow.keras.layers import Conv2DTranspose
from tensorflow.keras.layers import LeakyReLU
from tensorflow.keras.layers import Activation
from tensorflow.keras.layers import Flatten
from tensorflow.keras.layers import Dense
from tensorflow.keras.layers import Reshape
from tensorflow.keras.layers import Input
from tensorflow.keras.models import Model
from tensorflow.keras import backend as K
import numpy as np

class ConvAutoencoder:
    @staticmethod
    def build(width, height, depth, filters=(32, 64), latentDim=16):
        ### Encoder ###
        # Initialize the input shape to be 'channels last' along with the channels dimension itself
        inputShape = (height, width, depth)
        chanDim = -1

        # Define the input to the encoder
        inputs = Input(shape=inputShape)
        x = inputs

        # Loop over the number of filters
        for f in filters:
            # Apply CONV => RELU => BN operation
            x = Conv2D(f, (3, 3), strides=2, padding='same')(x)
            x = LeakyReLU(alpha=0.2)(x)
            x = BatchNormalization(axis=chanDim)(x)

        # Flatten the network and then construct our latent vector
        volumeSize = K.int_shape(x)
        x = Flatten()(x)
        latent = Dense(latentDim, name='encoded')(x)

        ### Decoder ###
        # Start building the decoder model which will accept the output of the encoder as its input
        x = Dense(np.prod(volumeSize[1:]))(latent)
        x = Reshape((volumeSize[1], volumeSize[2], volumeSize[3]))(x)

        # Loop over out number of filters again, but this time in reverse order
        for f in filters[::-1]:
            # Apply CONV_TRANSPOSE => RELU => BN operation
            x = Conv2DTranspose(f, (3, 3), strides=2, padding='same')(x)
            x = LeakyReLU(alpha=0.2)(x)
            x = BatchNormalization(axis=chanDim)(x)

        # Apply a single CONV_TRANSPOSE layer used to recover the original depth of the image
        x = Conv2DTranspose(depth, (3, 3), padding='same')(x)
        outputs = Activation('sigmoid', name='decoded')(x)

        # Construct autoencoder model
        autoencoder = Model(inputs, outputs, name='autoencoder')

        autoencoder.summary()

        # Return the autoencoder model
        return autoencoder