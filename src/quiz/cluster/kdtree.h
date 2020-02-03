/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

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

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
};
