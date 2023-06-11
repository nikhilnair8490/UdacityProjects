// detect corners in an image and store the resulting corner map in a file
#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>

// this function illustrates a very simple non-maximum suppression to extract the strongest corners
// in a local neighborhood around each pixel
cv::Mat PerformNMS(cv::Mat corner_img)
{
    // define size of sliding window
    int sw_size = 7;                  // should be odd so we can center it on a pixel and have symmetry in all directions
    int sw_dist = floor(sw_size / 2); // number of pixels to left/right and top/down to investigate

    // create output image
    cv::Mat result_img = cv::Mat::zeros(corner_img.rows, corner_img.cols, CV_8U);

    // loop over all pixels in the corner image
    for (int r = sw_dist; r < corner_img.rows - sw_dist - 1; r++) // rows
    {
        for (int c = sw_dist; c < corner_img.cols - sw_dist - 1; c++) // cols
        {
            // loop over all pixels within sliding window around the current pixel
            unsigned int max_val{0}; // keeps track of strongest response
            for (int rs = r - sw_dist; rs <= r + sw_dist; rs++)
            {
                for (int cs = c - sw_dist; cs <= c + sw_dist; cs++)
                {
                    // check wether max_val needs to be updated
                    unsigned int new_val = corner_img.at<unsigned int>(rs, cs);
                    max_val = max_val < new_val ? new_val : max_val;
                }
            }

            // check wether current pixel is local maximum
            if (corner_img.at<unsigned int>(r, c) == max_val)
                result_img.at<unsigned int>(r, c) = max_val;
        }
    }
  	  
    // visualize results
    std::string windowName = "NMS Result Image";
    cv::namedWindow(windowName, 5);
    cv::imshow(windowName, result_img);
    cv::waitKey(0);
  
    return result_img;
}

int main()
{
    // read corner image from file
    cv::Mat corner_img;
    corner_img = cv::imread("../images/img_circles.png");
  	if(corner_img.empty())
    {
        std::cout << "Could not read the image" << std::endl;
        return 1;
    }

    cv::cvtColor(corner_img, corner_img, cv::COLOR_BGR2GRAY);

    // perform simple non-maximum suppression
    cv::Mat nms_img = PerformNMS(corner_img);

    // save result to file
    cv::imwrite("../images/img_circles_nms.png", nms_img);
}