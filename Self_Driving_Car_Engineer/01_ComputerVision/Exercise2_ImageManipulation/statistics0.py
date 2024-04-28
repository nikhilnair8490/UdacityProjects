import glob
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from PIL import Image

def calculate_mean_std(image_list):
    """
    calculate mean and std of image list
    args:
    - image_list [list[str]]: list of image paths
    returns:
    - mean [array]: 1x3 array of float, channel wise mean
    - std [array]: 1x3 array of float, channel wise std
    """
    # IMPLEMENT THIS FUNCTION
    mean_arr = []
    std_arr = []
    for image_path in image_list:
        img = np.array(Image.open(image_path).convert('RGB'))
        R, G, B = img[..., 0], img[..., 1], img[..., 2]
        r_mean, g_mean, b_mean = np.mean(R), np.mean(G), np.mean(B)
        r_std, g_std, b_std = np.std(R), np.std(G), np.std(B)
        mean_arr.append([r_mean,g_mean,b_mean])
        std_arr.append([r_std, g_std, b_std])
    #print(np.array(mean_arr))
    mean = np.mean(np.array(mean_arr), axis=0)
    std = np.mean(np.array(std_arr), axis=0)
    return mean, std


def channel_histogram(image_list):
    """
    calculate channel wise pixel value
    args:
    - image_list [list[str]]: list of image paths
    """
    # IMPLEMENT THIS FUNCTION
    # Go through each image and calculate channel wise histogram
    r_hist = []
    g_hist = [] 
    b_hist = []
    for image_path in image_list:
        img = np.array(Image.open(image_path).convert('RGB'))
        R, G, B = img[..., 0], img[..., 1], img[..., 2]
        r_hist.extend(R.flatten().tolist())
        g_hist.extend(G.flatten().tolist())
        b_hist.extend(B.flatten().tolist())
        
    # Plot histogram of all channels using seaborn kde plot
    plt.figure()
    plt.title("Channel wise histogram")
    plt.xlabel("Pixel value")
    plt.ylabel("Density")
    sns.kdeplot(r_hist, color='r')
    sns.kdeplot(g_hist, color='g')
    sns.kdeplot(b_hist, color='b')
    plt.show()
     

if __name__ == "__main__": 
    image_list = glob.glob('data/images/*')
    mean, std = calculate_mean_std(image_list)
    channel_histogram(image_list[:2])
    print("Mean =", mean)
    print("Std =", std)