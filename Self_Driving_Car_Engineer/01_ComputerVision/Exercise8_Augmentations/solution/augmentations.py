import argparse
from functools import partial

import albumentations as A
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
from tensorflow.keras.preprocessing import image_dataset_from_directory

from utils import plot_batch


transforms = A.Compose([A.Rotate(limit=30, p=0.5),
                        A.Blur(blur_limit=5, p=0.5)])


def aug_fn(image):
    """ augment an image """
    aug_data = transforms(image=image.squeeze())
    aug_img = aug_data["image"]
    aug_img = tf.cast(aug_img/255.0, tf.float32)
    return aug_img


def process_data(image, label):
    """ wrapper function to apply augmentation """
    aug_img = tf.numpy_function(func=aug_fn, inp=[image], Tout=tf.float32)
    return aug_img, label


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Augment dataset')
    parser.add_argument('-d', '--imdir', required=True, type=str,
                        help='data directory')
    args = parser.parse_args()    

    dataset = image_dataset_from_directory(args.imdir, 
                                           image_size=(32, 32),
                                           validation_split=0.1,
                                           subset='training',
                                           seed=123,
                                           batch_size=1)
    dataset = dataset.map(process_data).batch(256)
    for X,Y in dataset:
        batch_np = X.numpy()
        plot_batch(batch_np)
        break
    print("See results by opening Desktop...")
        