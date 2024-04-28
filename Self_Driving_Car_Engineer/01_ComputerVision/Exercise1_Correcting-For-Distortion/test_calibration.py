import pickle
import cv2
import numpy as np

def solution(img, objpoints, imgpoints):
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1::-1], None, None)
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    return undist

def test(student_answer):
    # Read in the saved objpoints and imgpoints
    dist_pickle = pickle.load( open( "wide_dist_pickle.p", "rb" ) )
    objpoints = dist_pickle["objpoints"]
    imgpoints = dist_pickle["imgpoints"]
    
    # Read in an image
    img = cv2.imread('test_image.png')
    
    # Get undistorted answer
    answer = solution(img, objpoints, imgpoints)
    
    if np.isclose(img, student_answer).all():
        feedback = 'Oops! looks like no distortion correction was applied!'
    elif np.isclose(answer, student_answer).all():
        feedback = 'Correct! Nice work.'
    else:
        feedback = 'Oops! that looks different than my result!'
    
    return feedback
