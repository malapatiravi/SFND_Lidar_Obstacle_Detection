/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#ifndef CLUSTER_H
#define CLUSTER_H
#include "render/render.h"
#include "render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"
#include <unordered_set>

template <typename PointT>
void proximity2D(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree, float distanceTol,
				 std::vector<bool> &visited_set, std::vector<int> &cluster, int i, int maxSize)
{
	if (cluster.size() < maxSize)
	{
		cluster.push_back(i);
		visited_set[i] = true;
		std::vector<float> point{cloud->at(i).x, cloud->at(i).x, cloud->at(i).z};

		std::vector<int> nearby_points = tree->search(point, distanceTol);
		for (int it = 0; it < nearby_points.size(); it++)
		{
			if (!visited_set[nearby_points[it]])
			{
				proximity2D<PointT>(cloud, tree, distanceTol, visited_set, cluster, nearby_points[it], maxSize);
			}
			if (cluster.size() >= maxSize)
			{
				return;
			}
		}
	}
}
template <typename PointT>
void euclideanCluster2D(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree,
												 float distanceTol, int maxSize, int minSize, std::vector<pcl::PointIndices> &clusters)
{

	// TODO: Fill out this function to return list of indices for each cluster

	//std::vector<pcl::PointIndices> clusters; //Lis tof clusters
	std::vector<bool> visited_set(cloud->size(), false);
	for (int i_p = 0; i_p < cloud->size(); i_p++)
	{
		//Check if this point has not been visited
		//std::unordered_set<int>::iterator it = visited_set.find(i_p);
		if (!visited_set[i_p])
		{
			std::vector<int> clusterIndices;
			
			//this is not visited so visit now
			proximity2D<PointT>(cloud, tree, distanceTol, visited_set, clusterIndices, i_p, maxSize);
			if(clusterIndices.size() > minSize)
			{
				pcl::PointIndices cluster;
				cluster.indices = clusterIndices;
				clusters.push_back(cluster);
			}

			
		}
	}
	//return clusters;
}

void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int> &cluster, std::vector<bool> &processed, KdTree *tree, int distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);
	std::vector<int> nearest = tree->search(points[indice], distanceTol);
	for (int id : nearest)
	{
		if (!processed[id])
		{
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
		}
	}
}
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while (i < points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}
	return clusters;
}

#endif