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

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // Detector Type
    string detectorType = "ORB"; // SHITOMASI,HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    //Descriptor Type
    string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    //Matcher Type
    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
    string descriptorCat = "DES_BINARY"; // DES_BINARY, DES_HOG
    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

    // Stats for task MP7,8,9
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

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// TASK MP.1 -> Implement ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);
        // Go to end of the for loop to see implementation of Task MP.1

        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        bool bVisKeyPt = false;
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

        //// TASK MP.3 -> only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150); // Origin point of rect is top left corner
        if (bFocusOnVehicle)
        {
            for(auto i = keypoints.begin(); i != keypoints.end();) //No ++i here
            {
                if (!(vehicleRect.contains(i->pt)))
                {
                    i = keypoints.erase(i); // Return value is iterator pointing to next element
                }
                else
                {
                    ++i; //Increment iterator only if keyPt is inside bbox
                }
            }
        
        cout << "Number of keypoints in the bounding box: " << keypoints.size() << endl;

        }

        executionTime_KeyPtsArr.push_back(executionTime_KeyPtDet);
        numKeyPtsArr.push_back(keypoints.size());        

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

        // visualize keypoint detection results
        if (false)
        {
            cv::Mat visImage = img.clone();
            cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            string windowName = detectorType;
            cv::namedWindow(windowName, 6);
            imshow(windowName, visImage);
            cv::waitKey(0);
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, executionTime_Desc);

        //cout << "Descriptor Time: " << executionTime_Desc << endl;
        executionTime_DescArr.push_back(executionTime_Desc);

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;

            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorCat, matcherType, selectorType, executionTime_Matcher, numMatches);

            //cout << "matcher time: " << executionTime_Matcher << endl;
            //cout << "# of matches: " << numMatches << endl;
            executionTime_MatcherArr.push_back(executionTime_Matcher);
            numMatchesArr.push_back(numMatches);

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = false;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

        //// TASK MP.1 -> Implement ring buffer of size dataBufferSize
        // If the buffer size exceeds the limit then
        // remove the oldest image from buffer 
        if (dataBuffer.size() >= dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }

        //cout << " Databuffer size after:" << dataBuffer.size();

    } // eof loop over all images

    // Stats for Task MP7,8,9
    cout<< " All Image KeyPoint Numbers: "<< endl;
    for(double i: numKeyPtsArr)
    {
        cout<< i << " ";
    }
    cout<<"\n .................... \n";

    cout<< " All Image KeyPoint Execution Times: "<< endl;
    for(double i: executionTime_KeyPtsArr)
    {
        cout<< i << " ";
    }
    cout<<"\n .................... \n";

    cout<< " All Image Descriptor Execution Times: "<< endl;
    for(double i: executionTime_DescArr)
    {
        cout<< i << " ";
    }
    cout<<"\n .................... \n";

    cout<< " All Image Match Numbers: "<< endl;
    for(double i: numMatchesArr)
    {
        cout<< i << " ";
    }
    cout<<"\n .................... \n";

    cout<< " All Image Match Execution Times: "<< endl;
    for(double i: executionTime_MatcherArr)
    {
        cout<< i << " ";
    }
    cout<<"\n .................... \n";

    return 0;
}