/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H
#define KDTREE_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"
#include <chrono>
#include <string>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	bool isInDistance(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		float x_dist = point[0] - target[0];
		float y_dist = point[1] - target[1];
		float z_dist = point[2] - target[2];

		return ((x_dist * x_dist) + (y_dist * y_dist) + (z_dist * z_dist)) <= (distanceTol * distanceTol);
	}
	bool isInBox(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		bool inBox = true;

		inBox = inBox && (point[0] <= (target[0] + distanceTol)) && (point[0] >= (target[0] - distanceTol));
		inBox = inBox && (point[1] <= (target[1] + distanceTol)) && (point[1] >= (target[1] - distanceTol));
		inBox = inBox && (point[2] <= (target[2] + distanceTol)) && (point[2] >= (target[2] - distanceTol));
		return inBox;
	}
	void insertRecHelper(Node **rootP, std::vector<float> point, int id, int depth)
	{
		if (*rootP == NULL)
		{
			*rootP = new Node(point, id);
		}
		else
		{

			//Compare Y
			if (point[depth % 2] < (*rootP)->point[depth % 2])
			{
				insertRecHelper(&(*rootP)->left, point, id, depth + 1);
			}
			if (point[depth % 2] >= (*rootP)->point[depth % 2])
			{
				insertRecHelper(&(*rootP)->right, point, id, depth + 1);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertRecHelper(&root, point, id, 0);
	}


	void searchRecHelper(std::vector<float> target, float distanceTol, Node *node, int depth, std::vector<int> &ids)
	{
		if (node != NULL)
		{
			if (isInBox(target, node->point, distanceTol))
			{
				if (isInDistance(target, node->point, distanceTol))
				{
					ids.push_back(node->id);
				}
			}

			if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
			{
				searchRecHelper(target, distanceTol, node->left, depth + 1, ids);
			}
			if ((target[depth % 2] + distanceTol) > node->point[depth % 2])
			{
				searchRecHelper(target, distanceTol, node->right, depth + 1, ids);
			}
		}
		else
		{
			return;
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchRecHelper(target, distanceTol, root, 0, ids);
		return ids;
	}
};


#endif