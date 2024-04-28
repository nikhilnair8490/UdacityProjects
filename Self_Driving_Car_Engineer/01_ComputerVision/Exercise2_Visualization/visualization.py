import os
import matplotlib.pyplot as plt
from matplotlib.image import imread
from utils import get_data


def viz(ground_truth):
    """
    create a grid visualization of images with color coded bboxes
    args:
    - ground_truth [list[dict]]: ground truth data
    """
    # IMPLEMENT THIS FUNCTION

    # Set the path to your image folder
    folder_path = "data/images"

    # Get a list of all files in the folder
    image_files = [
        f for f in os.listdir(folder_path) if f.endswith((".png", ".jpg", ".jpeg"))
    ]

    # Set the number of columns for the grid
    num_columns = 5

    # Calculate the number of rows based on the number of images and columns
    num_rows = (len(image_files) + num_columns - 1) // num_columns

    # Create a grid layout
    fig, axes = plt.subplots(num_rows, num_columns, figsize=(12, 8))

    # Loop through each image file
    for j, image_file in enumerate(image_files):
        # Construct the full path to the image
        image_path = os.path.join(folder_path, image_file)

        # Read the image using matplotlib
        image = imread(image_path)

        # Get the ground truth data for the current image
        image_gt = [gt for gt in ground_truth if gt["filename"] == image_file]

        # Calculate the row and column index in the grid
        row_index = j // num_columns
        col_index = j % num_columns

        # Draw rectangle patches on the image for each bounding box and give different color to each class
        for gt in image_gt:
            for i, bbox in enumerate(gt["boxes"]):
                # Get the bounding box top-left corner coordinates and bottom-right corner coordinates
                y_top, x_top, y_bottom, x_bottom = bbox[0], bbox[1], bbox[2], bbox[3]
                # Convert to width and height
                x, y, w, h = x_top, y_top, x_bottom - x_top, y_bottom - y_top
                # Get the class label
                label = gt["classes"][i]

                # Assign string to class label
                if label == 1:
                    label_str = "Car"
                elif label == 2:
                    label_str = "Person"

                # Draw the bounding box and rotate edge colors based on the class label integer value
                # and add the label as text above the bounding box

                axes[row_index, col_index].text(
                    x, y, label_str, color=plt.cm.tab10(label), fontsize=8
                )

                # Add rectangle patch to the image
                axes[row_index, col_index].add_patch(
                    plt.Rectangle(
                        (x, y),
                        w,
                        h,
                        fill=False,
                        edgecolor=plt.cm.tab10(label),
                        linewidth=2,
                        label=label,
                    )
                )

        # Add the image in the corresponding grid cell
        axes[row_index, col_index].imshow(image)

    # Add a title for the entire grid
    fig.suptitle(
        "Grid layout of images with bounding boxes and class labels", fontsize=10
    )
    # Display the final grid image
    plt.show()


if __name__ == "__main__":
    ground_truth, _ = get_data()
    viz(ground_truth)
