/* \author Aaron Brown */
/* Nikhil Nair: Implemented 3D RANSAC function for plane*/
// Quiz on implementing simple RANSAC plane fitting

#include "ransac3d.h"

/**
 * @brief RANSAC function for 3d plane
 *
 * @param cloud - point cloud
 * @param maxIterations - maximum number of iterations
 * @param distanceTol -  distance tolerance for finding inliers
 * @return std::unordered_set<int> -  set of inlier indices
 */
template <typename PointT>
std::unordered_set<int> Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
		PointT p1 = cloud->points[rand() % cloud->points.size()];
		PointT p2 = cloud->points[rand() % cloud->points.size()];
		PointT p3 = cloud->points[rand() % cloud->points.size()];

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
