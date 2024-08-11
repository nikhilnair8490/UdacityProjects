import argparse
import logging

import tensorflow as tf
from tensorflow.keras.layers import Dense, Flatten, Conv2D, MaxPooling2D, AveragePooling2D

from utils import get_datasets, get_module_logger, display_metrics


def create_network(input_shape):
    # Implement the LeNet-5 Model (modified)
    lenet_5_model = tf.keras.Sequential()
    # C1 Convolution Layer
    lenet_5_model.add(Conv2D(6, kernel_size=5, strides= 1, activation = 'relu', input_shape= input_shape, padding='same'))
    # S2 Sub Sampling Layer
    lenet_5_model.add(MaxPooling2D(pool_size=(2,2),strides=(2,2)))
    # C3 Convolution Layer
    lenet_5_model.add(Conv2D(16, kernel_size=5, strides= 1, activation = 'relu', padding='valid'))
    # S4 SubSampling Layer
    lenet_5_model.add(MaxPooling2D(pool_size=(2,2),strides=(2,2)))
    # C5 Convolution Layer
    lenet_5_model.add(Conv2D(120, kernel_size=5, strides= 1, activation = 'relu', padding='valid'))
    # Flatten for FF NN
    lenet_5_model.add(Flatten())
    # F6 Dense fully connected layer
    lenet_5_model.add(Dense(128, activation='relu'))
    # F7 Dense fully connected layer
    lenet_5_model.add(Dense(256, activation='relu'))    
    # Output Layer
    lenet_5_model.add(Dense(43))
    return lenet_5_model


if __name__  == '__main__':
    logger = get_module_logger(__name__)
    parser = argparse.ArgumentParser(description='Download and process tf files')
    parser.add_argument('-d', '--imdir', required=True, type=str,
                        help='data directory')
    parser.add_argument('-e', '--epochs', default=10, type=int,
                        help='Number of epochs')
    args = parser.parse_args()    

    logger.info(f'Training for {args.epochs} epochs using {args.imdir} data')
    # get the datasets
    train_dataset, val_dataset = get_datasets(args.imdir)

    model = create_network(input_shape=(32,32,3))

    model.compile(optimizer='adam',
              loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])
    history = model.fit(x=train_dataset, 
                        epochs=args.epochs, 
                        validation_data=val_dataset)
    display_metrics(history)