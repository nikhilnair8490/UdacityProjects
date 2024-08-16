import matplotlib.pyplot as plt
import numpy as np


def plot_batch(batch):
    indices = np.random.choice(range(256), replace=False, size=10)
    f, ax = plt.subplots(2, 5, figsize=(15, 5))
    for i, idx in enumerate(indices):
        x = i // 5
        y = i % 5
        im = batch[idx, ...]
        im *= 255
        im = im.astype(np.uint8)
        ax[x, y].imshow(im)
    plt.tight_layout
    plt.show()