# **LiDAR Obstacle Detection**

<img src="media/25_WorkingwithRealPCD_StreamPCDs_4.gif" width="100%" height="100%">

\
**LiDAR (Light Detection and Ranging)** uses eye-safe laser beams to “see” the world in 3D, providing machines and computers with an accurate representation of the surveyed environment. This 3D map is called a point cloud. An onboard computer can utilize the lidar point cloud for safe navigation.

Autonomous vehicles rely on lidar sensors to detect and identify obstacles and to generate 3D maps of the surrounding environment.

Lidar technology used in autonomous vehicles typically includes high-resolution sensors that can detect objects at long distances and provide detailed information about the object's size, shape, and material composition. Lidar sensors work by emitting laser pulses and measuring the time it takes for the pulses to bounce back after hitting an object. By analyzing the data from multiple lidar sensors, autonomous vehicles can create a precise 3D map of the environment and accurately detect and classify objects in real-time.

Compared to other perception sensors such as cameras and radars, lidar has several advantages for autonomous vehicles. Lidar sensors can provide accurate and reliable data in various weather and lighting conditions, which is critical for safe operation. Lidar sensors also offer high resolution and accuracy in detecting objects, enabling autonomous vehicles to make quick and informed decisions. Additionally, lidar can help autonomous vehicles navigate complex environments, such as urban areas, where multiple obstacles and objects are present.

However, lidar sensors are expensive and require a lot of power to operate. In addition, lidar sensors are not as widely available as cameras and radars, which can limit the adoption of lidar technology in autonomous vehicles.

---
## **Project Overview**

In this project i apply all the concepts that have learned for processing point clouds through the course, and use it to detect car and trucks on a narrow street using lidar. The detection pipeline follow the below methods,

- Filtering (Voxel Grid Filter)
- Cropping (Region of Interest)
- Plane Segmentation ([RANSAC](Sensor_Fusion_Engineer/Lidar_Obstacle_Detection/src/quiz/ransac)) 
- Clustering ([Euclidean Clustering](Sensor_Fusion_Engineer/Lidar_Obstacle_Detection/src/quiz/cluster))
- Bounding Boxes
    
The segmentation, and clustering methods are created from scratch and are my own implementation. The remaining methods are implemented using PCL library. Follow through the embedded links to learn more about the methods.

---
## Setting up the workspace

1. Tools used in this project:

    - C++ compiler. I prefer using linux, it already has gcc preinstalled.
    
    - You will need to have PCL installed on your system. You can find instructions for installing PCL here: https://pointclouds.org/downloads/#linux
    
    - To compile and build the project, you will need to have cmake and make installed on your system. You can find instructions for installing cmake here: https://cmake.org/install/
    
    I have used the following versions for this project:  
    - Ubuntu 20.04 LTS
    - C++ 14
    - PCL 1.10
    - gcc 9.4.0
    - CMake 3.16.3


2. Clone this repository to your local machine
```bash
cd ~  
git clone https://github.com/nikhilnair8490/UdacityProjects.git
```
3. Go to the project folder

`UdacityProjects/Sensor_Fusion_Engineer/Lidar_Obstacle_Detection/`

4. Make a build directory in this folder: 
```bash
mkdir build && cd build
```
5. Run the following commands to build & run the project:
 ```bash
CMake ..
make
./environment
```
6. You should now see the visualization of the point cloud data stream.

