from tensorflow import keras
from tensorflow.keras.layers import Input, Conv2D, MaxPooling2D, UpSampling2D, ZeroPadding2D
from tensorflow.keras.models import Model
from tensorflow.keras.callbacks import TensorBoard
from tensorflow.keras.datasets import mnist
import numpy as np

### Loading MNIST dataset ###
(x_train, y_train), (x_test, y_test) = mnist.load_data()

x_train = x_train.astype('float32') / 255   
x_test = x_test.astype('float32') / 255

x_train = np.reshape(x_train, (len(x_train), 28, 28, 1)) # Adapt this if using 'channels_first' image data format
x_test = np.reshape(x_test, (len(x_test), 28, 28, 1)) # Adapt this if using 'channels_first' image data format

# Generate dataset with gaussain noise
noise_factor = 0.5

x_train_noisy = x_train + noise_factor * np.random.normal(loc=0.0, scale=1.0, size=x_train.shape)
x_test_noisy = x_test + noise_factor * np.random.normal(loc=0.0, scale=1.0, size=x_test.shape)

x_train_noisy = np.clip(x_train_noisy, 0., 1.)
x_test_noisy = np.clip(x_test_noisy, 0., 1.)

### Training model ###
def train_model():

    # Autoencoder convolution NN model
    input_img = Input(shape=(28, 28, 1)) # Adapt this if using 'channels_first' image data format
    x = Conv2D(16, (3, 3), activation='relu', padding='same')(input_img)
    x = MaxPooling2D((2,2), padding='same')(x)
    x = Conv2D(8, (3, 3), activation='relu', padding='same')(x)
    x = MaxPooling2D((2, 2), padding='same')(x)
    x = Conv2D(8, (3, 3), activation='relu', padding='same')(x)
    encoded = MaxPooling2D((2, 2), padding='same', name='encoder')(x)

    # At this point the representation is (4, 4, 8) --> 128-dimensional

    # Decoder convolution NN model
    x = Conv2D(8, (3, 3), activation='relu', padding='same')(encoded)
    x = UpSampling2D((2, 2))(x)
    x = Conv2D(8, (3, 3), activation='relu', padding='same')(x)
    x = UpSampling2D((2, 2))(x)
    x = Conv2D(16, (3, 3), activation='relu')(x)
    x = UpSampling2D((2, 2))(x)
    decoded = Conv2D(1, (3, 3), activation='sigmoid', padding='same')(x)

    # Declare autoencoder convolution NN model
    autoencoder = Model(input_img, decoded)

    # Build autoencoder convolution NN model
    autoencoder.compile(optimizer='adadelta', loss='binary_crossentropy')

    # Apply input dataset and output dataset
    # Start training autoencoder convolution NN model
    # Define training & validatio properties
    autoencoder.fit(
        x_train_noisy, x_train,
        epochs=20,
        batch_size=128,
        shuffle=True,
        validation_data=(x_test_noisy, x_test),
        callbacks=[TensorBoard(log_dir='/tmp/tb', histogram_freq=0, write_graph=False)]
    )
    autoencoder.save('autoencoder.h5')

train_model()