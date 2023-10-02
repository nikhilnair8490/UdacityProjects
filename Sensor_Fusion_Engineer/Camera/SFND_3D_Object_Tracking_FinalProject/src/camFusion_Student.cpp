
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/*
 * The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size.
 * However, you can make this function work for other sizes too.
 * For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
 */
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if (bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
}

/**
 * @brief Match list of 3D objects (vector<BoundingBox>) between current and previous frame
 *
 * @param matches       List of best matches between previous and current frame
 * @param bbBestMatches Output list of best matches between previous and current frame (matched boxID pairs)
 * @param prevFrame     Previous frame
 * @param currFrame     Current frame
 */
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    /* STEP 1 : Go through all keypoint matches and associate them with their respective bounding boxes in both images */

    std::multimap<int, int> bbTempMatches;

    // Loop over all the keypoint match pairs
    for (auto matchIt = matches.begin(); matchIt != matches.end(); ++matchIt)
    {
        int prevImgBoxID = -1;
        int currImgBoxID = -1;

        // Loop through all the BBoxes in previous image and find the box ID of the 'query' keypoint(match keypoint in previous image)
        for (auto it = prevFrame.boundingBoxes.begin(); it != prevFrame.boundingBoxes.end(); ++it)
        {
            cv::KeyPoint keyPt;
            keyPt = prevFrame.keypoints[matchIt->queryIdx];
            if (it->roi.contains(keyPt.pt))
            // if ((keyPt.pt.x > (it->roi.x)) && (keyPt.pt.x < (it->roi.x + it->roi.width)) &&
            //     (keyPt.pt.y > (it->roi.y)) && (keyPt.pt.y < (it->roi.y + it->roi.height)))
            {
                it->keypoints.push_back(keyPt);
                prevImgBoxID = it->boxID;
                break;
            }
        }

        // Loop through all the BBoxes in current image and find the box ID of the 'train' keypoint(match keypoint in current image)
        for (auto it = currFrame.boundingBoxes.begin(); it != currFrame.boundingBoxes.end(); ++it)
        {
            cv::KeyPoint keyPt;
            keyPt = currFrame.keypoints[matchIt->trainIdx];
            if (it->roi.contains(keyPt.pt))
            // if ((keyPt.pt.x > (it->roi.x)) && (keyPt.pt.x < (it->roi.x + it->roi.width)) &&
            //     (keyPt.pt.y > (it->roi.y)) && (keyPt.pt.y < (it->roi.y + it->roi.height)))
            {
                it->keypoints.push_back(keyPt);
                currImgBoxID = it->boxID;
                break;
            }
        }

        // Store the box ID pairs in a temporary multimap
        if ((prevImgBoxID != -1) && (currImgBoxID != -1)) // Exlcude pairs which are not part of either BBoxes
        {
            bbTempMatches.insert(std::make_pair(prevImgBoxID, currImgBoxID));
        }

    } // eof Loop over all keypoint match pairs

    /* STEP 2: For each BBox pair count the number of keypoint matches*/

    // Find all the unique keys (BBox IDs in the prev image) from the multimap
    std::set<int> unique_keys;
    int last_key = INT_MIN; // some value that won't appear

    for (auto it = bbTempMatches.begin(); it != bbTempMatches.end(); ++it)
    {
        if (it->first != last_key)
        {
            unique_keys.insert(it->first);
            last_key = it->first;
        }
    }

    // Display contents of multimap
    if (false)
    {
        for (auto itr = bbTempMatches.begin(); itr != bbTempMatches.end(); ++itr)
        {
            cout << '\t' << itr->first << '\t' << itr->second
                 << '\n';
        }
    }

    // Create a map to count occurrences of each key-value pair
    std::map<std::pair<int, int>, int> count_map;

    // Loop through each element in the multimap
    for (auto it = bbTempMatches.begin(); it != bbTempMatches.end(); ++it)
    {
        // Create a pair from the key and value of multimap
        std::pair<int, int> key_value_pair = std::make_pair(it->first, it->second);

        // Check if the pair already exists in count_map
        if (count_map.find(key_value_pair) == count_map.end())
        {
            // If not, insert it with a count of 1
            count_map.insert(std::make_pair(key_value_pair, 1));
        }
        else
        {
            // If it exists, increment the count
            count_map[key_value_pair]++;
        }
    }

    // Display the count of each key-value pair
    if (false)
    {
        for (auto it = count_map.begin(); it != count_map.end(); ++it)
        {
            std::cout << "(" << it->first.first << ", " << it->first.second << "): " << it->second << std::endl;
        }
    }

    /* STEP 3: The BBox pair with highest number of keypoint match occurences is the best matched BBox pair*/

    // Iterate through each unique bounding box IDs in the previous image
    for (auto it = unique_keys.begin(); it != unique_keys.end(); ++it)
    {
        int BBoxIdx1 = -1; // BBox index
        int BBoxIdx2 = -1;
        int maxKyPtCnt = INT_MIN;

        // Loop through all the BBox matched pairs and find the ones with highest keypoint occurences
        for (auto it1 = count_map.begin(); it1 != count_map.end(); ++it1)
        {
            int currKyPtCnt = it1->second; // Number of occurences for the current BBox pair

            if (it1->first.first == *it)
            {
                if (currKyPtCnt >= maxKyPtCnt)
                {
                    maxKyPtCnt = currKyPtCnt;
                    BBoxIdx1 = it1->first.first;
                    BBoxIdx2 = it1->first.second;
                }
            }
        }

        if ((BBoxIdx1 != -1) && (BBoxIdx2 != -1)) // Exlcude pairs which are not part of either BBoxes
        {
            bbBestMatches.insert(std::make_pair(BBoxIdx1, BBoxIdx2));
        }
    }

    // Display the count of each BBox pair
    if (false)
    {
        for (auto it = bbBestMatches.begin(); it != bbBestMatches.end(); ++it)
        {
            std::cout << "(" << it->first << ", " << it->second << "): " << std::endl;
        }
    }
}
