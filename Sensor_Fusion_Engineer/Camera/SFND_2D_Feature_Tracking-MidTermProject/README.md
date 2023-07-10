# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

---
## **Project Overview**
This mid term project builds the feature tracking algorithm and test various detector / descriptor combinations to see which ones perform best. The mid-term project consists of four parts:

* First, it focuses on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, it integrates several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, the focus is on descriptor extraction and matching using brute force and also the FLANN approach. 
* In the last part, once the code framework is complete, it tests the various algorithms in different combinations and compare them with regard to some performance measures.

---
## Setting up the workspace

1. Tools used in this project:

    - C++ compiler. I prefer using linux, it already has gcc preinstalled.
    
    - OpenCV >= 4.1
      * All OSes: refer to the [official instructions](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html)
      * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors. If using [homebrew](https://brew.sh/): `$> brew install --build-from-source opencv` will install required dependencies and compile opencv with the `opencv_contrib` module by default (no need to set `-DOPENCV_ENABLE_NONFREE=ON` manually). 
      * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
    
    - To compile and build the project, you will need to have cmake and make installed on your system. You can find instructions for installing cmake here: https://cmake.org/install/
    
    I have used the following versions for this project:  
    - Ubuntu 20.04 LTS
    - C++ 14
    - OpenCV 4.5.0
    - gcc 9.4.0
    - CMake 3.16.3


2. Clone this repository to your local machine
```bash
cd ~  
git clone https://github.com/nikhilnair8490/UdacityProjects.git
```
1. Go to the project folder

`UdacityProjects/Sensor_Fusion_Engineer/Camera/SFND_2D_Feature_Tracking-MidTermProject`

4. Make a build directory in this folder: 
```bash
mkdir build && cd build
```
5. Run the following commands to build & run the project:
 ```bash
CMake ..
make
./2D_feature_tracking
```
6. You should now see the visualization.
