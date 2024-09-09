import json

from utils import calculate_iou, check_results


def nms(predictions):
    """
    non max suppression
    args:
    - predictions [dict]: predictions dict 
    returns:
    - filtered [list]: filtered bboxes and scores
    """
    filtered = []
    
    boxes = predictions['boxes']
    scores = predictions['scores']
    
    sorted_boxes = [x for _,x in sorted(zip(scores, boxes), key=lambda x:x[0], reverse = True)]
    sorted_scores = sorted(scores, reverse=True)
    
    i = 0  # Start index for the outer loop
    while i < len(sorted_boxes):
        current_box = sorted_boxes[i]
        
        # Inner loop to compare with remaining elements
        j = i + 1  # Start from the next element
        while j < len(sorted_boxes):
            compare_box = sorted_boxes[j]
            iou = calculate_iou(current_box, compare_box)

            if iou >= 0.5:
                sorted_boxes.pop(j)  # Remove element at index j
                sorted_scores.pop(j)
            else:
                j += 1  # Move to the next element if not popped
        
        i += 1  # Move to the next element for the outer loop)
        
    filtered =[[x,y] for x,y in zip(sorted_boxes, sorted_scores)]
    
    # IMPLEMENT THIS FUNCTION
    return filtered


if __name__ == '__main__':
    with open('data/predictions_nms.json', 'r') as f:
        predictions = json.load(f)
        print(predictions)
    
    filtered = nms(predictions)
    check_results(filtered)