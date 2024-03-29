
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
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, size_t &imgIndex, bool bWait)
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
        putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 0.8, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 0.8, currColor);
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

    // Save images in output file if true
    if (false)
    {
        // Specify the file name and format (PNG in this case)
        std::string filenameOut1 = "TopViewImage_" + std::to_string(imgIndex) + ".png";

        // Save the image to the specified file
        bool success = cv::imwrite(filenameOut1, topviewImg);
    }
    else
    {
        if (bWait)
        {
            cv::waitKey(0); // wait for key to be pressed
        }
    }
}

/**
 * @brief Cluster keypoint matches with the current bounding box
 *
 * @param boundingBox   Current bounding box
 * @param kptsPrev      Previous frame keypoints
 * @param kptsCurr      Current frame keypoints
 * @param kptMatches    Keypoint matches between previous and current frame
 */
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    /* STEP 1: Associate the keypoint match with the bounding box and calculate keypoint match distance*/

    // Loop through all the keypoint matches and find the ones which are part of the current bounding box
    // Also find the distance between the matched keypoints
    std::vector<double> distKptMatches;
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint keyPtPrev, keyPtCurr;
        double dist;
        keyPtPrev = kptsPrev[it->queryIdx];
        keyPtCurr = kptsCurr[it->trainIdx];

        if (boundingBox.roi.contains(keyPtCurr.pt))
        {
            boundingBox.kptMatches.push_back(*it);
            dist = cv::norm(keyPtCurr.pt - keyPtPrev.pt);
            distKptMatches.push_back(dist);
        }
    }

    /*STEP 2: Remove outlier keypoint matches from the bounding box*/

    // Get the Q1 and Q3 percentile for the distance vector
    double q1 = percentile(distKptMatches, 0.25);
    double q3 = percentile(distKptMatches, 0.75);
    // Find the IQR for the distance vector
    double iqr = q3 - q1;

    // Go through all the matched keypoint pairs in the current bounding box and remove the ones which are outliers from the bounding box
    auto it1 = boundingBox.kptMatches.begin();
    while (it1 != boundingBox.kptMatches.end())
    {
        cv::KeyPoint keyPtPrev, keyPtCurr;
        double dist;
        keyPtPrev = kptsPrev[it1->queryIdx];
        keyPtCurr = kptsCurr[it1->trainIdx];
        dist = cv::norm(keyPtCurr.pt - keyPtPrev.pt);

        if ((dist < (q1 - 1.5 * iqr)) || (dist > (q3 + 1.5 * iqr)))
        {
            it1 = boundingBox.kptMatches.erase(it1); // erase() returns the next iterator
        }
        else
        {
            ++it1;
        }
    }
}

/**
 * @brief Compute time-to-collision (TTC) based on keypoint correspondences in successive images
 * 
 * @param kptsPrev      Previous frame keypoints
 * @param kptsCurr      Current frame keypoints
 * @param kptMatches    Keypoint matches between previous and current frame
 * @param frameRate     Frame rate of the camera 
 * @param TTC           Output TTC
 * @param visImg        Output visualization image
 */
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    /* STEP 1: Compute distance ratios between all matched keypoints*/

    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint match  loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint match loop

            double minDist = 100.0; // min. required distance in pixels

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    /* STEP 2: Filter out outliers from the distance ratio vector*/

    // Remove the outliers from the distance ratio vector
    std::vector<double> filtDistRatios = removeOutliers(distRatios);
    // Copy original distance ratio vector to filtered distance ratio vector
    // filtDistRatios = distRatios;

    /* STEP 3: Compute camera-based TTC from distance ratios*/

    // compute camera-based TTC from mean distance ratios
    double meanDistRatio = std::accumulate(filtDistRatios.begin(), filtDistRatios.end(), 0.0) / filtDistRatios.size();

    double dT = 1 / frameRate;
    // TTC = -dT / (1 - meanDistRatio);

    // Alternate method to compute camera-based TTC from distance ratios using median
    double medianDistRatio;

    std::sort(filtDistRatios.begin(), filtDistRatios.end());
    if (distRatios.size() % 2 == 0)
    {
        medianDistRatio = (filtDistRatios[filtDistRatios.size() / 2 - 1] + filtDistRatios[filtDistRatios.size() / 2]) / 2;
    }
    else
    {
        medianDistRatio = filtDistRatios[filtDistRatios.size() / 2];
    }

    TTC = -dT / (1 - medianDistRatio);

    // Print the mean and median distance ratios
    if (false)
    {
        std::cout << "meanDistRatio = " << meanDistRatio << " medianDistRatio = " << medianDistRatio << std::endl;
    }
}

/**
 * @brief Compute time-to-collision (TTC) based on Lidar measurements
 *
 * @param lidarPointsPrev Previous Lidar points
 * @param lidarPointsCurr Current Lidar points
 * @param frameRate       Frame rate of the camera
 * @param TTC             Output TTC
 */
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC, double &velLidar)
{
    // auxiliary variables
    double dT = 1.0 / frameRate; // time between two measurements in seconds
    std::vector<double> lidarPointsPrevX, lidarPointsCurrX;
    std::vector<double> filtLidarPointsPrevX, filtLidarPointsCurrX;

    // Create vector of all x points from previous and current Lidar points
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        lidarPointsPrevX.push_back(it->x);
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        lidarPointsCurrX.push_back(it->x);
    }

    // Filter out outliers from the X-distance vector
    filtLidarPointsPrevX = removeOutliers(lidarPointsPrevX);
    filtLidarPointsCurrX = removeOutliers(lidarPointsCurrX);

    // find closest distance to Lidar points within ego lane
    double minXPrev = INT_MAX, minXCurr = INT_MAX;
    for (auto it = filtLidarPointsPrevX.begin(); it != filtLidarPointsPrevX.end(); ++it)
    {
        minXPrev = minXPrev > *it ? *it : minXPrev;
    }

    for (auto it = filtLidarPointsCurrX.begin(); it != filtLidarPointsCurrX.end(); ++it)
    {
        minXCurr = minXCurr > *it ? *it : minXCurr;
    }

    // Print both the minimum X distances in same line
    if (false)
    {
        std::cout << "minXPrev = " << minXPrev << " minXCurr = " << minXCurr << std::endl;
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
    // Compute velocity
    velLidar = (minXPrev - minXCurr) / dT;
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
        if ((prevImgBoxID != -1) && (currImgBoxID != -1)) // Exclude pairs which are not part of either BBoxes
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

        if ((BBoxIdx1 != -1) && (BBoxIdx2 != -1)) // Exclude pairs which are not part of either BBoxes
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

/* Helper Functions */

/**
 * @brief Function to calculate the percentile of a sorted dataset
 *
 * @param dataIn    Input dataset
 * @param p         Percentile
 * @return double   Value of the percentile
 */
double percentile(const std::vector<double> &dataIn, double p)
{
    std::vector<double> data = dataIn;
    std::sort(data.begin(), data.end());

    int N = data.size();
    double n = (N - 1) * p + 1;

    // If n is an integer, then percentile is a data point
    if (n == floor(n))
    {
        return data[n - 1];
    }
    else
    {
        int k = floor(n);
        double d = n - k;
        return data[k - 1] + d * (data[k] - data[k - 1]);
    }
}

/**
 * @brief Function to remove outliers based on IQR
 *
 * @param data                  Input dataset
 * @return std::vector<double>  Filtered dataset
 */
std::vector<double> removeOutliers(const std::vector<double> &data)
{
    std::vector<double> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());

    double q1 = percentile(sorted_data, 0.25);
    double q3 = percentile(sorted_data, 0.75);

    double iqr = q3 - q1;

    std::vector<double> filtered_data;
    for (double x : data)
    {
        if (x >= q1 - 1.5 * iqr && x <= q3 + 1.5 * iqr)
        {
            filtered_data.push_back(x);
        }
    }

    return filtered_data;
}