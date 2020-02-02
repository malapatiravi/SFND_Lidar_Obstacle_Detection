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
	int dimension;
	KdTree()
		: root(NULL)
	{
		dimension = 2;
	}

	void insertHelper(Node **node, std::vector<float> point, int id, int level)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint cd = level % 2;
			if (point[cd] < ((*node)->point[cd]))
			{
				insertHelper(&((*node)->left), point, id, level + 1);
			}
			else
			{
				insertHelper(&((*node)->right), point, id, level + 1);
			}

			// /* code */
			// if (level % 2 == 1) /* Compare Y*/
			// {
			// 	if((*root)->point[1] >= point[1])
			// 	{
			// 		insertHelper(&(*root)->right, point, id, level+1);
			// 	}
			// 	else
			// 	{
			// 		/* code */
			// 		insertHelper(&(*root)->left, point, id, level+1);
			// 	}
			// }
			// else /* Compare X*/
			// {
			// 	/* code */
			// 	if((*root)->point[0] >= point[0])
			// 	{
			// 		insertHelper(&(*root)->right, point, id, level+1);
			// 	}
			// 	else
			// 	{
			// 		/* code */
			// 		insertHelper(&(*root)->left, point, id, level+1);

			// 	}
			// }
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, point, id, 0);
		return;
	}
	void set_dimension(int new_dimension)
	{
		dimension = new_dimension;
	}

	int get_split_by_index(int depth)
	{
		return (depth % dimension);
	}
	bool within_sphere(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		float x_dist = point[0] - target[0];
		float y_dist = point[1] - target[1];
		float z_dist = point[2] - target[2];

		return ((x_dist * x_dist) + (y_dist * y_dist) + (z_dist * z_dist)) <= (distanceTol * distanceTol);
	}

	bool within_box(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		bool within_box = true;
		for (int i = 0; i < target.size(); ++i)
		{
			within_box = within_box && (point[i] <= (target[i] + distanceTol)) && (point[i] >= (target[i] - distanceTol));
		}
		return within_box;
	}
	//void searchHelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
	void searchHelper(std::vector<int> &neighbors, std::vector<float> target, float distanceTol, Node *node, int depth)
	{

		if (node == nullptr)
		{
			return;
		}
		int index = get_split_by_index(depth);

		if (within_box(target, node->point, distanceTol) && within_sphere(target, node->point, distanceTol))
		{
			neighbors.push_back(node->id);
		}
		if ((target[index] - distanceTol) < node->point[index])
		{
			searchHelper(neighbors, target, distanceTol, node->left, depth + 1);
		}
		if ((target[index] + distanceTol) > node->point[index])
		{
			searchHelper(neighbors, target, distanceTol, node->right, depth + 1);
		}

		// if(node!=NULL)
		// {
		// 	if( (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol) )&&
		// 		(node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol) ) )
		// 	{
		// 		float distance = sqrt( (node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1]));
		// 		if(distance <= distanceTol)
		// 		{
		// 			ids.push_back(node->id);
		// 		}
		// 	}
		// }

		// if((target[depth%2]-distanceTol) < node->point[depth%2])
		// {
		// 	searchHelper(target, node->left, depth+1, distanceTol, ids);
		// }
		// if((target[depth%2]+distanceTol)> node->point[depth%2])
		// {
		// 	searchHelper(target, node->right, depth+1, distanceTol, ids);
		// }
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(ids, target, distanceTol, root, 0);

		return ids;
	}
};
