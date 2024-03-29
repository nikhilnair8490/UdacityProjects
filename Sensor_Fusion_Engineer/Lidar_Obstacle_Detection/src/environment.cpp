/* \author Aaron Brown */
// Nikhil Nair: Create new functions for process and render real PCD data
// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
    pointCloud = lidar->scan();

    // Instantiate point processor object
    ProcessPointClouds<pcl::PointXYZ> *processPointClouds = new ProcessPointClouds<pcl::PointXYZ>();

    // Segment the cloud in two parts Plane cloud (for road) and Obstacle cloud (for anything thats not road)
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processPointClouds->SegmentPlane(pointCloud, 50, 0.2);

    // Create clusters from the obstacles cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processPointClouds->Clustering(segmentCloud.first, 1, 3, 30);

    int renderView = 3;
    switch (renderView)
    {
    case 0:
    {
        // Render lidar rays
        renderRays(viewer, lidar->position, pointCloud);
        break;
    }
    case 1:
    {
        // Render point cloud data
        renderPointCloud(viewer, pointCloud, "pointCloud");
        break;
    }
    case 2:
    {
        // Render the point cloud with segmentation
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
        break;
    }
    case 3:
    {
        // Render the cluster point cloud for all obstacles
        int clusterId = 0;
        std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

        for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            std::cout << "cluster size ";
            processPointClouds->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
            // Create and render bounding box around each cluster
            Box box = processPointClouds->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
            ++clusterId;
        }
        break;
    }
    default:
        // Render point cloud data
        renderPointCloud(viewer, pointCloud, "pointCloud");
    }

    delete lidar;
    delete processPointClouds;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //  Filter the input cloud using pcl voxel filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2f, Eigen::Vector4f(-10, -5, -3, 1), Eigen::Vector4f(30, 6, 0.8, 1));

    // Segment the cloud in two parts Plane cloud (for road) and Obstacle cloud (for anything thats not road)
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 50, 0.2);

    // Create clusters from the obstacles cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.4, 10, 2000);

    int renderView = 3;
    switch (renderView)
    {
    case 0:
    {
        // Render raw input cloud
        renderPointCloud(viewer, inputCloud, "inputCloud");
        break;
    }
    case 1:
    {
        // Render filtered cloud
        renderPointCloud(viewer, filterCloud, "filterCloud");
        break;
    }
    case 2:
    {
        // Render the point cloud with segmentation
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
        break;
    }
    case 3:
    {
        // Render the cluster point cloud for all obstacles
        int clusterId = 0;
        std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 1), Color(0, 0, 1)};

        for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
            // Create and render bounding box around each cluster
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
            ++clusterId;
        }
        // Render the point cloud with ground plane
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
        break;
    }
    default:
        // Render raw input cloud
        renderPointCloud(viewer, inputCloud, "inputCloud");
    }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    // ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000001.pcd");
    // cityBlock(viewer, pointProcessorI, inputCloud);
    // // To render static frames
    //  while (!viewer->wasStopped())
    //  {
    //      viewer->spinOnce();
    //  }

    // Create point processor and point cloud objects for streaming
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    // Render streaming point cloud data
    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}