# Exercise 2 - Visualization

## Objective

For this exercise, you need to implement a function to visualize the ground truth boxes
on a set of images in `visualization.py`. You need to display color coded bounding boxes using the class id associated
with each bounding box. You need to display all the data in a single figure.
You should aim for visibility as clear data visualization is critical to communicate a message.

![](example.png)

**Note:** Any visualized code will only pop up through the workspace desktop - if you complete work in the primary workspace window, you'll need to click the "Desktop" button in the bottom right to view visualizations.

## Details

The labels (bounding boxes and classes) are located is the `data/ground_truth.json` file. It contains 20 observations, each observation is a dict
with the following fields.

```
{filename: str, boxes: List[List[int]], classes: List[int]}
```
The bounding boxes are using the `[x1, y1, x2, y2]` format. Images (png files) are located in the `data/images` folder. Each image is associated can be matched with its labels with the filename. 

The `utils.py` file contains an help function `get_data` that you can import to load the ground truth and the predictions. You will only need the ground truth for 
this exercise though. 

## Tips
You can use matplotlib patches to create the bounding boxes visualizations. You can improve over the above visualization by adding the classes name by the bounding boxes. 

## Take it further
Think about the way you could display both ground truths and predictions on the same image in a clear fashion.