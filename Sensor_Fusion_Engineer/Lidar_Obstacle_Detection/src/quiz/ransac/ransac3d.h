#ifndef RANSAC3D_H_
#define RANSAC3D_H_

/**
 * @file ransac3d.h
 * @author Nikhil Nair
 * @brief Header file for all the functions used in the RANSAC3D quiz
 * @version 0.1
 * @date 2023-03-24
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <unordered_set>
#include "../../processPointClouds.h"

template <typename PointT>
std::unordered_set<int> Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

#endif /* RANSAC3D_H_ */