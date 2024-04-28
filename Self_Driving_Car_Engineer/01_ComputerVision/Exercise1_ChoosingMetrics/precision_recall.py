import numpy as np

from iou import calculate_ious
from utils import get_data


def precision_recall(ious, gt_classes, pred_classes):
    """
    calculate precision and recall
    args:
    - ious [array]: NxM array of ious
    - gt_classes [array]: 1xN array of ground truth classes
    - pred_classes [array]: 1xM array of pred classes
    returns:
    - precision [float]
    - recall [float]
    """
    # IMPLEMENT THIS FUNCTION
    noOfTPs = 0
    noOfFPs = 0
    noOfFNs = 0
    matchCnt = 0
    precision = 0
    recall = 0
    for i, row in enumerate(ious):
        for j, iou_val in enumerate(row):
            #print(iou_val)
            if (iou_val > 0.5):
                matchCnt+=1
                if (gt_classes[i] == pred_classes[j]):
                    noOfTPs+=1
                else:
                    noOfFPs+=1
                
    noOfFNs = len(gt_classes) - matchCnt
    precision = noOfTPs/(noOfTPs+noOfFPs)
    recall = noOfTPs/(noOfTPs+noOfFNs)  
    print('Precision=' ,precision)       
    print('Recall=',recall)
    
    return precision, recall


if __name__ == "__main__": 
    ground_truth, predictions = get_data()
    
    # get bboxes array
    filename = 'segment-1231623110026745648_480_000_500_000_with_camera_labels_38.png'
    gt_bboxes = [g['boxes'] for g in ground_truth if g['filename'] == filename][0]
    gt_bboxes = np.array(gt_bboxes)
    gt_classes = [g['classes'] for g in ground_truth if g['filename'] == filename][0]
    

    pred_bboxes = [p['boxes'] for p in predictions if p['filename'] == filename][0]
    pred_boxes = np.array(pred_bboxes)
    pred_classes = [p['classes'] for p in predictions if p['filename'] == filename][0]
    
    ious = calculate_ious(gt_bboxes, pred_boxes)
    precision, recall = precision_recall(ious, gt_classes, pred_classes)