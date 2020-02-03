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
	bool isInBox(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		bool inBox = true;
		for(int i = 0; i < target.size(); i++)
		{
			inBox = inBox && (point[i] <= (target[i] + distanceTol)) && (point[i] >= (target[i]-distanceTol));
		}
		return inBox;
	}
	bool isInDistance(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		float x_dist = point[0]-target[0];
		float y_dist = point[1]-target[1];
		float z_dist = point[2]-target[2];

		return ((x_dist*x_dist)+ (y_dist*y_dist) + (z_dist*z_dist) ) <= (distanceTol*distanceTol);


	}
	void searchRecHelper(std::vector<float> target, float distanceTol, Node* node, int depth, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			if(isInBox(target, node->point, distanceTol))
			{
				if(isInDistance(target, node->point, distanceTol))
				{
					ids.push_back(node->id);
				}
			}

			if((target[depth%2] - distanceTol) < node->point[depth%2])
			{
				searchRecHelper(target, distanceTol, node->left, depth+1, ids);
			}
			if((target[depth%2] + distanceTol) > node->point[depth%2])
			{
				searchRecHelper(target, distanceTol, node->right, depth+1, ids);
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


		return ids;
	}
};
