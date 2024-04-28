import cv2
import numpy as np
import matplotlib.pyplot as plt


def create_mask(path, color_threshold):
    """
    create a binary mask of an image using a color threshold
    args:
    - path [str]: path to image file
    - color_threshold [array]: 1x3 array of RGB value
    returns:
    - img [array]: RGB image array
    - mask [array]: binary array
    """
    # IMPLEMENT THIS FUNCTION
    # Read image from path
    img = cv2.imread(path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # Create a mask using color threshold
    mask = cv2.inRange(img, np.array(color_threshold), np.array([255, 255, 255]))

    return img, mask


def mask_and_display(img, mask):
    """
    display 3 plots next to each other: image, mask and masked image
    args:
    - img [array]: HxWxC image array
    - mask [array]: HxW mask array
    """
    # IMPLEMENT THIS FUNCTION
    # Mask the image
    masked_img = cv2.bitwise_and(img, img, mask=mask)

    f, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(10, 5))
    ax1.imshow(img)
    ax1.set_title("Original Image", fontsize=10)
    ax2.imshow(mask)
    ax2.set_title("+ Mask", fontsize=10)
    ax3.imshow(masked_img)
    ax3.set_title("= Masked Image", fontsize=10)
    plt.show()


if __name__ == "__main__":
    path = "data/images/segment-1231623110026745648_480_000_500_000_with_camera_labels_38.png"
    color_threshold = [128, 128, 128]
    img, mask = create_mask(path, color_threshold)
    mask_and_display(img, mask)
