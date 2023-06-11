#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    //std::cout<< "dst.size() = " << dst << std::endl;
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);

    // Locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    // of the type `vector<cv::KeyPoint>`.

    std::vector<cv::KeyPoint> keypoints;
    // Loop throught the harris response matrix and find maxima
    double maxOverlap = 0.0;
    for (size_t j =0; j <= dst_norm.rows; ++j)
    {
        for (size_t i = 0; i <= dst_norm.cols; ++i)
        {
            int response = (int)dst_norm.at<float>(j,i);
            if (response > minResponse) // only store points above a threshold as keypoints
            {
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i,j);
                newKeyPoint.size = 2*apertureSize;
                newKeyPoint.response = response;

                //Find if there is overlap between the new keypoint and the existing keypoints
                bool Overlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    // If there is overlap, check if the new keypoint has a higher response
                    if (kptOverlap > maxOverlap)
                    {
                        Overlap = true;
                        // If the new keypoint has a higher response, replace the old keypoint with the new one
                        if (newKeyPoint.response > (*it).response)
                        {
                            *it = newKeyPoint;
                            break;
                        }
                    }
                }
                // If there is no overlap, add the new keypoint to the list
                if (!Overlap)
                {
                    keypoints.push_back(newKeyPoint);
                }
            }
        }
    }

    // visualize keypoints
    windowName = "Harris Corner Detection Results";
    cv::namedWindow(windowName, 5);
    cv::Mat newKeyPtImg = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, keypoints, newKeyPtImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, newKeyPtImg);
    cv::waitKey(0);

}

int main()
{
    cornernessHarris();
}