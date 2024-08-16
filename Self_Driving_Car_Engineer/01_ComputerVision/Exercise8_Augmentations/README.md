# Exercise 3 - Augmentations

## Objective

In this exercise, you will experiment with the [Albumentations](https://albumentations.ai/docs/) library
to perform different data augmentations. 

## Details

Write down a list of relevant augmentations and store them in the `transforms` variable. You should also
implement a quick script to visualize the batches and check your augmentations.

You can run `python augmentations.py` to display augmented images (in the Desktop window).

## Tips

You should use the `Compose` API to use multiple augmentations. You can find an example of an augmentation
pipeline using `Compose` [here](https://albumentations.ai/docs/examples/example/#define-an-augmentation-pipeline-using-compose-pass-the-image-to-it-and-receive-the-augmented-image).

This [Github repository](https://github.com/albumentations-team/albumentations_examples)
contains different examples of augmentations.