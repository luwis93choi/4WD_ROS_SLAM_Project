import numpy as numpy
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers


### Graphs of Layers Creation ###
inputs = keras.Input(shape=(784,))

dense = layers.Dense(64, activation='relu')
x = dense(inputs)

x = layers.Dense(64, activation='relu')(x)
outputs = layers.Dense(10)(x)

model = keras.Model(inputs=inputs, outputs=outputs, name='mnist_model')

### Model Summary Display ###
model.summary()

### Plotting NN Model ###
keras.utils.plot_model(model, 'my_first_model.png', show_shapes=True)

### Load Training Dataset ###
(x_train, y_train), (x_test, y_test) = keras.datasets.mnist.load_data()

x_train = x_train.reshape(60000, 784).astype('float32') / 255
x_test = x_test.reshape(10000, 784).astype('float32') / 255

### Compile the model (Define learning properties of the model) ###
model.compile(
    loss = keras.losses.SparseCategoricalCrossentropy(from_logits=True),
    optimizer = keras.optimizers.RMSprop(),
    metrics=['accuracy'],
)

### Define Training Parameters and Start Training ###
history = model.fit(x_train, y_train, batch_size=64, epochs=2, validation_split=0.2)

### Evaluate and Save the Results of Learning ###
test_scores = model.evaluate(x_test, y_test, verbose=2)

print('Test Loss : ', test_scores[0])
print('Test Accuracy : ', test_scores[1])

### Save NN model as a package ###
model.save('path_to_my_model')
del model

### Reload saved NN model ###
model = keras.models.load_model('path_to_my_model')