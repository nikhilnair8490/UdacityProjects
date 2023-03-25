/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
// Nikhil Nair: Implement euclidean cluster and proximity function

#include "kdtree.h"
#include <unordered_set>

/**
 * @brief Function to find nearest neighbors of a point in a point cloud
 *
 * @param point Target point to find neighbors
 * @param cluster Add all neighbors to cluster
 * @param distanceTol Distance tolerance to form a cluster
 * @param tree k-d tree structure for points
 * @param processedPoints Set of points that have been processed
 */
void proximity(const std::vector<std::vector<float>> &points, int index, std::vector<int> &cluster, float distanceTol, KdTree2 *tree, std::unordered_set<int> &processedPoints)
{
	// Add point to processed
	processedPoints.insert(index);
	// Add point to cluster
	cluster.push_back(index);

	// Find neighbors of the point in the cluster
	std::vector<int> neighbors = tree->search(points[index], distanceTol);
	// Iterate through the neighbor points and find their neighbors
	for (int i = 0; i < neighbors.size(); ++i)
	{
		if (processedPoints.find(neighbors[i]) == processedPoints.end())
		{
			proximity(points, neighbors[i], cluster, distanceTol, tree, processedPoints);
		}
	}
	return;
}

/**
 * @brief Find all the euclidean clusters for given point cloud
 *
 * @param points Input point cloud
 * @param tree k-d tree structure for points
 * @param distanceTol Distance tolerance to form a cluster
 * @return std::vector<std::vector<int>> List of indices for each cluster
 */
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, KdTree2 *tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;

	// Create an empty unordered set
	std::unordered_set<int> processedPoints;

	// Iterate through each point in points
	for (int i = 0; i < points.size(); ++i)
	{
		// If the point has not been processed
		if (processedPoints.find(i) == processedPoints.end())
		{
			// Create a new cluster
			std::vector<int> cluster;
			// Call the proximity function to find nearest neighbors
			proximity(points, i, cluster, distanceTol, tree, processedPoints);
			// Add the cluster to the list of clusters
			clusters.push_back(cluster);
		}
	}

	return clusters;
}
