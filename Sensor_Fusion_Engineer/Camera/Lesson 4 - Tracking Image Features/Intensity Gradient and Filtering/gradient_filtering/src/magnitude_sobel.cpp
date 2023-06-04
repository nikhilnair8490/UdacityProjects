#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void magnitudeSobel()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1gray.png");

    // convert image to grayscale
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // apply smoothing using the GaussianBlur() function from the OpenCV
    cv::Mat imgGray_Smooth;
    cv::GaussianBlur(imgGray, imgGray_Smooth, cv::Size(5, 5), 2.0);
    // Get the gaussian kernel for debugging
    cv::Mat kernel_gauss = cv::getGaussianKernel(25, 2.0);
    //std::cout << "kernel_gauss = " << kernel_gauss << std::endl;

    // create filter kernels using the cv::Mat datatype both for x and y
    // Sobel X,Y direction filters
    float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2, 
                        -1, 0, +1};
    cv::Mat kernel_x = cv::Mat(3, 3, CV_32F, sobel_x);

    float sobel_y[9] = {-1, -2, -1,
                         0,  0,  0, 
                        +1, +2, +1};
    cv::Mat kernel_y = cv::Mat(3, 3, CV_32F, sobel_y);   

    // apply filter using the OpenCv function filter2D()
    cv::Mat grad_x, grad_y;
    cv::filter2D(imgGray_Smooth, grad_x, -1, kernel_x, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv::filter2D(imgGray_Smooth, grad_y, -1, kernel_y, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    
    // compute magnitude image based on the equation presented in the lesson 
    //cv::Mat magnitude = cv::Mat(grad_x.rows, grad_x.cols, CV_32F);
    //cv::Mat grad_sum;
    //cv:add(grad_x.mul(grad_x), grad_y.mul(grad_y), grad_sum, cv::noArray(), CV_32F);
    //cv::sqrt(grad_sum, magnitude);
    // Convert to 8 bit unsigned data type
    //magnitude.convertTo(magnitude, CV_8U);

    // compute magnitude image
    cv::Mat magnitude = imgGray.clone();
    for (int r = 0; r < magnitude.rows; r++)
    {
        for (int c = 0; c < magnitude.cols; c++)
        {
            magnitude.at<unsigned char>(r, c) = std::sqrt(std::pow(grad_x.at<unsigned char>(r, c), 2) +
                                                     std::pow(grad_y.at<unsigned char>(r, c), 2));
        }
    }

    //std::cout << "magnitude = " << magnitude << std::endl;
    

    // show result
    string windowName = "Gaussian Blurring";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, magnitude);
    cv::waitKey(0); // wait for keyboard input before continuing
}

int main()
{
    magnitudeSobel();
}