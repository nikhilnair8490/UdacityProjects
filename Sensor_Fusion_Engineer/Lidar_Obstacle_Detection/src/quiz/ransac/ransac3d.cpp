/* \author Aaron Brown */
/* Nikhil Nair: Implemented RANSAC function for plane*/
// Quiz on implementing simple RANSAC plane fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(-16, -16, 16, 1, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

/**
 * @brief RANSAC function for 3d plane
 *
 * @param cloud - point cloud
 * @param maxIterations - maximum number of iterations
 * @param distanceTol -  distance tolerance for finding inliers
 * @return std::unordered_set<int> -  set of inlier indices
 */
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliersResultTemp;
	srand(time(NULL));

	int numofInliers;
	for (size_t i = 0; i <= maxIterations; ++i)
	{
		// Randomly sample three points from the given point cloud
		pcl::PointXYZ p1 = cloud->points[rand() % cloud->points.size()];
		pcl::PointXYZ p2 = cloud->points[rand() % cloud->points.size()];
		pcl::PointXYZ p3 = cloud->points[rand() % cloud->points.size()];

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		x1 = p1.x;
		y1 = p1.y;
		z1 = p1.z;
		x2 = p2.x;
		y2 = p2.y;
		z2 = p2.z;
		x3 = p3.x;
		y3 = p3.y;
		z3 = p3.z;

		// Check if all the points are non-collinear
		// Area of triangle formed from these points should be non-zero
		if ((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) == 0)
		{
			continue;
		}

		// Fit a plane between the points p1, p2 and p3
		// General equation of plane (Ax + By + Cz + D = 0)
		float A = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
		float B = ((z2 - z1) * (x3 - x1)) - ((x2 - x1)) * ((z3 - z1));
		float C = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
		float D = -(A * x1 + B * y1 + C * z1);

		float d; // Shortest distance between point and plane
		float num;
		float den;
		float x;
		float y;
		float z;

		// Loop through all the points in the cloud
		for (size_t j = 0; j < cloud->points.size(); ++j)
		{
			// Point cloud coordinate
			x = cloud->points[j].x;
			y = cloud->points[j].y;
			z = cloud->points[j].z;

			// Shortest distance between the plane Ax+ By + Cz + D = 0 and the point (x,y,z)
			num = fabs(A * x + B * y + C * z + D);
			den = sqrt(A * A + B * B + C * C);
			d = num / den;

			// If d is less than distance threshold then add the point as inlier
			if (d < distanceTol)
			{
				inliersResultTemp.insert(j);
			}
		}

		// If the current number of inliers are more than max inliers,then update the max inliers result
		if (inliersResultTemp.size() > inliersResult.size())
		{
			inliersResult = inliersResultTemp;
		}
		inliersResultTemp.clear();
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "quiz/RANSAC Plane took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// Get inliers using Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 3D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
