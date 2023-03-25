// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include "./quiz/ransac/ransac3d.cpp"
#include "./quiz/cluster/cluster.cpp"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> voxelFilter;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    voxelFilter.setInputCloud(cloud);
    voxelFilter.setLeafSize(filterRes, filterRes, filterRes); // Voxel cube size
    voxelFilter.filter(*cloudFiltered);

    // Region based filtering. Remove cloud points outside a defined region
    typename pcl::PointCloud<PointT>::Ptr regionCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> regionFilter(true);
    regionFilter.setMin(minPoint);
    regionFilter.setMax(maxPoint);
    regionFilter.setInputCloud(cloudFiltered);
    regionFilter.filter(*regionCloud);

    // Filter out the points from roof of car
    std::vector<int> roofIndices;
    pcl::CropBox<PointT> roofFilter(true);
    roofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roofFilter.setInputCloud(regionCloud);
    roofFilter.filter(roofIndices);

    // Extract the points from the regionCloud
    pcl::PointIndices::Ptr roofInliers(new pcl::PointIndices);
    for (int point : roofIndices)
    {
        roofInliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extractIndices;
    extractIndices.setInputCloud(regionCloud);
    extractIndices.setIndices(roofInliers);
    extractIndices.setNegative(true);
    extractIndices.filter(*regionCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regionCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());

    // Loop through the indices in the inliers and add them in the planeCloud object
    for (int index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Extract the indices which are part of the inliers (Plane cloud) from the reference cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);   // set the reference cloud on which extraction is performed
    extract.setIndices(inliers);    // set the indices to be extracted
    extract.setNegative(true);      // set to true to remove the  points from input cloud
    extract.filter(*obstacleCloud); // Returns the modified input cloud with extracted indices

    // Output the pair of point clouds (obstacle cloud & plane cloud)
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

    bool usePCLlib = false;
    if (usePCLlib)
    {
        // Function to find inliers for the cloud. Inliers are  part of the segment plane (e.g road)
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);       // Data model for thr Sample Consensus algorithm
        seg.setMethodType(pcl::SAC_RANSAC);          // Random Sample Consensus algorithm
        seg.setDistanceThreshold(distanceThreshold); // Determines how close a point must be to the model in order to be considered an inlier
        seg.setMaxIterations(maxIterations);         // Maximum number of iterations for the Consensus

        // Segment the largest planar component from the input cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
    }
    else
    {
        // Use the custom 3D RANSAC function for plane segmentation
        std::unordered_set<int> inliersSet = Ransac3d<PointT>(cloud, maxIterations, distanceThreshold);

        // Convert the unordered set to pcl::PointIndices inliers to be used in the SeparateClouds function
        for (int index : inliersSet)
        {
            inliers->indices.push_back(index);
        }
    }

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset.\n"
                  << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    bool usePCLlib = false;
    if (usePCLlib)
    {
        // Function to perform euclidean clustering to group detected obstacles

        // Creating the KdTree object for the search method of the extraction
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        // Extract the indices related to all the clusters
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(minSize);
        ec.setMaxClusterSize(maxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // Go through each cluster and add them in a vector of cluster clouds 'clusters'
        for (const auto &cluster : cluster_indices)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            for (const auto &idx : cluster.indices)
            {
                cloud_cluster->points.push_back(cloud->points[idx]);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
        }
    }
    else
    {
        // Use the custom KD-Tree function for euclidean clustering
        KdTree2 *tree2 = new KdTree2;
        std::vector<std::vector<float>> pointsVec;
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            // Store the cloud point in a float vector for matching the KD-Tree2 insert function
            std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            tree2->insert(point, i);
            pointsVec.push_back(point);
        }

        // Extract the indices related to all the clusters within clusterTolerance
        std::vector<std::vector<int>> clusterIndices2 = euclideanCluster(pointsVec, tree2, clusterTolerance);

        for (const auto &cluster : clusterIndices2)
        {
            // Check cluster size is between minSize and maxSize and continue if not
            if (cluster.size() >= minSize && cluster.size() <= maxSize)
            {

                typename pcl::PointCloud<PointT>::Ptr cloud_cluster2(new pcl::PointCloud<PointT>);
                for (const auto &idx : cluster)
                {
                    cloud_cluster2->points.push_back(cloud->points[idx]);
                }
                cloud_cluster2->width = cloud_cluster2->points.size();
                cloud_cluster2->height = 1;
                cloud_cluster2->is_dense = true;

                clusters.push_back(cloud_cluster2);
            }
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in ascending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}