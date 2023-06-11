// detect corners in an image and store the resulting corner map in a file

#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


void DetectCorners(std::string input_filename)
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/" + input_filename + ".png");
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
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // save result to file
    cv::imwrite("../images/" + input_filename + "_result.bmp",  dst_norm_scaled);
}

int main()
{
    // detect corners and store the result in a floating-point image
    std::string img_name = "img_traffic";
    DetectCorners(img_name);
}