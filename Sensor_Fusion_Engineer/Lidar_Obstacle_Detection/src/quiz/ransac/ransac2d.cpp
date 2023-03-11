/* \author Aaron Brown */
/* Nikhil Nair: Implemented RANSAC function*/
// Quiz on implementing simple RANSAC line fitting

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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

/**
 * @brief RANSAC function for 2d line
 *
 * @param cloud - point cloud
 * @param maxIterations - maximum number of iterations
 * @param distanceTol -  distance tolerance for finding inliers
 * @return std::unordered_set<int> -  set of inlier indices
 */
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliersResultTemp;
	srand(time(NULL));

	int numofInliers;
	int numofInlierMax = 0;
	for (size_t i = 0; i <= maxIterations; ++i)
	{
		// Randomly sample two points from the given point cloud
		pcl::PointXYZ p1 = cloud->points[rand() % cloud->points.size()];
		pcl::PointXYZ p2 = cloud->points[rand() % cloud->points.size()];

		// Fit a line between the points p1 and p2

		// Given two points: point1 (x1, y1) and point2 (x2, y2),
		// the line through point1 and point2 has the specific form:(y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)=0
		// Coefficients of the line (Ax + By + C = 0)
		float A = (p1.y - p2.y);
		float B = (p2.x - p1.x);
		float C = (p1.x * p2.y - p2.x * p1.y);

		float d; // Shortest distance between point and line
		float num;
		float den;
		float x;
		float y;

		// Loop through all the points in the cloud
		for (size_t j = 0; j < cloud->points.size(); ++j)
		{
			// Point cloud coordinate
			x = cloud->points[j].x;
			y = cloud->points[j].y;

			// Shortest distance between the line Ax+ By + C = 0 and the point
			num = abs(A * x + B * y + C);
			den = sqrt(A * A + B * B);
			d = num / den;

			// If d is less than distance threshold then add the point as inlier
			if (d < distanceTol)
			{
				inliersResultTemp.insert(j);
			}
		}

		numofInliers = inliersResultTemp.size();
		// If the current number of inliers are more than max then udpate max
		// and also update the inliers result
		if (numofInliers > numofInlierMax)
		{
			numofInlierMax = numofInliers;
			inliersResult = inliersResultTemp;
		}
		inliersResultTemp.clear();
	}

	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// Get inliers using Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 35, 2);

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

	// Render 2D point cloud with inliers and outliers
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
