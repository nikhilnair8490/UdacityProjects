
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;  // last file index to load
    int imgStepWidth = 1;
    int imgFillWidth = 4; // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    string yoloBasePath = dataPath + "dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4, 4, cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4, 4, cv::DataType<double>::type);        // rotation matrix and translation vector

    RT.at<double>(0, 0) = 7.533745e-03;
    RT.at<double>(0, 1) = -9.999714e-01;
    RT.at<double>(0, 2) = -6.166020e-04;
    RT.at<double>(0, 3) = -4.069766e-03;
    RT.at<double>(1, 0) = 1.480249e-02;
    RT.at<double>(1, 1) = 7.280733e-04;
    RT.at<double>(1, 2) = -9.998902e-01;
    RT.at<double>(1, 3) = -7.631618e-02;
    RT.at<double>(2, 0) = 9.998621e-01;
    RT.at<double>(2, 1) = 7.523790e-03;
    RT.at<double>(2, 2) = 1.480755e-02;
    RT.at<double>(2, 3) = -2.717806e-01;
    RT.at<double>(3, 0) = 0.0;
    RT.at<double>(3, 1) = 0.0;
    RT.at<double>(3, 2) = 0.0;
    RT.at<double>(3, 3) = 1.0;

    R_rect_00.at<double>(0, 0) = 9.999239e-01;
    R_rect_00.at<double>(0, 1) = 9.837760e-03;
    R_rect_00.at<double>(0, 2) = -7.445048e-03;
    R_rect_00.at<double>(0, 3) = 0.0;
    R_rect_00.at<double>(1, 0) = -9.869795e-03;
    R_rect_00.at<double>(1, 1) = 9.999421e-01;
    R_rect_00.at<double>(1, 2) = -4.278459e-03;
    R_rect_00.at<double>(1, 3) = 0.0;
    R_rect_00.at<double>(2, 0) = 7.402527e-03;
    R_rect_00.at<double>(2, 1) = 4.351614e-03;
    R_rect_00.at<double>(2, 2) = 9.999631e-01;
    R_rect_00.at<double>(2, 3) = 0.0;
    R_rect_00.at<double>(3, 0) = 0;
    R_rect_00.at<double>(3, 1) = 0;
    R_rect_00.at<double>(3, 2) = 0;
    R_rect_00.at<double>(3, 3) = 1;

    P_rect_00.at<double>(0, 0) = 7.215377e+02;
    P_rect_00.at<double>(0, 1) = 0.000000e+00;
    P_rect_00.at<double>(0, 2) = 6.095593e+02;
    P_rect_00.at<double>(0, 3) = 0.000000e+00;
    P_rect_00.at<double>(1, 0) = 0.000000e+00;
    P_rect_00.at<double>(1, 1) = 7.215377e+02;
    P_rect_00.at<double>(1, 2) = 1.728540e+02;
    P_rect_00.at<double>(1, 3) = 0.000000e+00;
    P_rect_00.at<double>(2, 0) = 0.000000e+00;
    P_rect_00.at<double>(2, 1) = 0.000000e+00;
    P_rect_00.at<double>(2, 2) = 1.000000e+00;
    P_rect_00.at<double>(2, 3) = 0.000000e+00;

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;                       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer;                 // list of data frames which are held in memory at the same time
    bool bVis = false;                            // visualize results
    bool bVisKeyPt = false;

    // Detector Type
    string detectorType = "SHITOMASI"; // SHITOMASI,HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    // Descriptor Type
    string descriptorType = "BRIEF"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT Note: AKAZE Descriptors only work with AKAZE Keypoints
    // Matcher Type
    string matcherType = "MAT_BF";       // MAT_BF, MAT_FLANN
    string descriptorCat = "DES_BINARY"; // DES_BINARY, DES_HOG Note: Use DES_HOG for SIFT and BINARY for rest
    string selectorType = "SEL_KNN";     // SEL_NN, SEL_KNN

    // Stats
    double executionTime_KeyPtDet;
    vector<double> executionTime_KeyPtsArr;
    int numKeyPts;
    vector<int> numKeyPtsArr;

    double executionTime_Desc;
    vector<double> executionTime_DescArr;

    double executionTime_Matcher;
    vector<double> executionTime_MatcherArr;
    int numMatches;
    vector<int> numMatchesArr;

    vector<double> TTC_LidarArr;
    vector<double> velLidarArr;
    vector<double> TTC_CameraArr;

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex += imgStepWidth)
    {
        // Image Index
        cout << "Image Index: " << imgIndex << endl;

        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file
        cv::Mat img = cv::imread(imgFullFilename);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = img;
        dataBuffer.push_back(frame);

        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT & CLASSIFY OBJECTS */
        bVis = false;
        float confThreshold = 0.2;
        float nmsThreshold = 0.4;
        detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
                      yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);

        cout << "#2 : DETECT & CLASSIFY OBJECTS done" << endl;

        /* CROP LIDAR POINTS */

        // load 3D Lidar points from file
        string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);

        // remove Lidar points based on distance properties
        float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
        cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

        (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

        cout << "#3 : CROP LIDAR POINTS done" << endl;

        /* CLUSTER LIDAR POINT CLOUD */

        // associate Lidar points with camera-based ROI
        float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
        clusterLidarWithROI((dataBuffer.end() - 1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

        // Visualize 3D objects
        bVis = true;
        if (bVis)
        {
            show3DObjects((dataBuffer.end() - 1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), imgIndex, true);
        }
        bVis = false;

        cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << endl;

        // REMOVE THIS LINE BEFORE PROCEEDING WITH THE FINAL PROJECT
        // continue; // skips directly to the next image without processing what comes beneath

        /* DETECT IMAGE KEYPOINTS */

        // convert current image to grayscale
        cv::Mat imgGray;
        cv::cvtColor((dataBuffer.end() - 1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, executionTime_KeyPtDet, bVisKeyPt);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, executionTime_KeyPtDet, bVisKeyPt);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType, executionTime_KeyPtDet, bVisKeyPt);
        }

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;

        cout << "#5 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        cv::Mat descriptors;
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, executionTime_Desc);

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#6 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType, executionTime_Matcher, numMatches);

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            /* TRACK 3D OBJECT BOUNDING BOXES */

            //// STUDENT ASSIGNMENT
            //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
            map<int, int> bbBestMatches;
            matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end() - 2), *(dataBuffer.end() - 1)); // associate bounding boxes between current and previous frame using keypoint matches
            //// EOF STUDENT ASSIGNMENT

            // continue;

            // store matches in current data frame
            (dataBuffer.end() - 1)->bbMatches = bbBestMatches;

            cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << endl;

            /* COMPUTE TTC ON OBJECT IN FRONT */

            // loop over all BB match pairs
            for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
            {
                // find bounding boxes associates with current match
                BoundingBox *prevBB, *currBB;
                for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                {
                    if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        currBB = &(*it2);
                    }
                }

                for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                {
                    if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        prevBB = &(*it2);
                    }
                }

                // compute TTC for current match
                if (currBB->lidarPoints.size() > 0 && prevBB->lidarPoints.size() > 0) // only compute TTC if we have Lidar points
                {
                    //// STUDENT ASSIGNMENT
                    //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                    double ttcLidar;
                    double velLidar;
                    computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar, velLidar);
                    //// EOF STUDENT ASSIGNMENT
                    if (false)
                    {
                        std::cout << "prevBB: " << prevBB->boxID << "currBB: " << currBB->boxID << std::endl;
                        std::cout << "TTC Lidar = " << ttcLidar << std::endl;
                    }
                    TTC_LidarArr.push_back(ttcLidar);
                    velLidarArr.push_back(velLidar);

                    //// STUDENT ASSIGNMENT
                    //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                    //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                    double ttcCamera;
                    clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches);

                    // visualize keypoint matches between current and previous image
                    bVis = false;
                    if (bVis)
                    {
                        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                        currBB->kptMatches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        cv::Mat resizeMatchImg;
                        cv::resize(matchImg, resizeMatchImg, cv::Size(), 0.70, 0.70);
                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, resizeMatchImg);
                        // Save images in output file if true
                        if (true)
                        {
                            // Specify the file name and format (PNG in this case)
                            std::string filenameOut0 = "KeyPtMatch_" + std::to_string(imgIndex) + ".png";

                            // Save the image to the specified file
                            bool success = cv::imwrite(filenameOut0, resizeMatchImg);
                        }
                        else
                        {
                            cout << "Press key to continue to next frame" << endl;
                            cv::waitKey(0);
                        }
                    }
                    bVis = false;

                    computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera);
                    // Print TTC Camera
                    if (false)
                    {
                        std::cout << "TTC Camera = " << ttcCamera << std::endl;
                    }
                    TTC_CameraArr.push_back(ttcCamera);
                    //// EOF STUDENT ASSIGNMENT

                    bVis = true;
                    if (bVis)
                    {
                        cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                        showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                        cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);

                        char str[200];
                        // sprintf(str, "Frame: %zu", imgIndex);
                        sprintf(str, "Frame: %zu, TTC Lidar : %f s, TTC Camera : %f s", imgIndex, ttcLidar, ttcCamera);
                        putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255));

                        string windowName = "Final Results : TTC";
                        cv::namedWindow(windowName, 4);
                        cv::imshow(windowName, visImg);

                        // Save images in output file if true
                        if (false)
                        {
                            // Specify the file name and format (PNG in this case)
                            std::string filenameOut = "Finalframe_" + std::to_string(imgIndex) + ".png";

                            // Save the image to the specified file
                            bool success = cv::imwrite(filenameOut, visImg);
                        }
                        else
                        {
                            cout << "Press key to continue to next frame" << endl;
                            cv::waitKey(0);
                        }
                    }
                    bVis = false;

                } // eof TTC computation
            }     // eof loop over all BB matches
        }

        // If the buffer size exceeds the limit then
        // remove the oldest image from buffer
        if (dataBuffer.size() >= dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }

    } // eof loop over all images

    // Stats for Task FP5

    // Print both TTC Lidar and velocity Lidar in same line with comma seperator
    cout << "\n TTC Lidar, velLidar " << endl;
    for (int i = 0; i < TTC_LidarArr.size(); i++)
    {
        cout << TTC_LidarArr[i] << ", " << velLidarArr[i] << "\n";
    }

    cout << "\n .................... \n";

    // Print TTC Camera
    cout << "\n TTC Camera " << endl;
    for (int i = 0; i < TTC_CameraArr.size(); i++)
    {
        cout << TTC_CameraArr[i] << " ";
    }

    cout << "\n .................... \n";

    return 0;
}
