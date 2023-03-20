/* \author Aaron Brown */
// Nikhil Nair: Implement insert function
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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// Function to insert a new point into the tree

		// Insert the point as root if the root is NULL
		if (root == NULL)
		{
			root = new Node(point, id);
			return;
		}
		else
		{
			// Create a new node from the point
			Node *newNode = new Node(point, id);
			// Traverse through the tree and find an empty slot to place the new point
			Node *temp = root; // Start from root node
			uint depth = 0;	   // depth of tree
			while (temp != NULL)
			{
				if (depth % 2 == 0) // tree depth for x split
				{
					if (point[0] < temp->point[0])
					{
						// Assign the next empty node on left branch
						if (temp->left == NULL)
						{
							temp->left = newNode;
							return;
						}
						temp = temp->left;
					}
					else
					{
						// Assign the next empty node on right branch
						if (temp->right == NULL)
						{
							temp->right = newNode;
							return;
						}
						temp = temp->right;
					}
				}
				else // tree depth for y split
				{
					if (point[1] < temp->point[1])
					{
						// Assign the next empty node on left branch
						if (temp->left == NULL)
						{
							temp->left = newNode;
							return;
						}
						temp = temp->left;
					}
					else
					{
						// Assign the next empty node on right branch
						if (temp->right == NULL)
						{
							temp->right = newNode;
							return;
						}
						temp = temp->right;
					}
				}
				// If there are no empty slots in current level go to next depth level of tree
				depth += 1;
			}
		}

		return;
	}

	// Recursive helper function to search for all nearest neighbors of target
	void searchRecursive(Node *node, std::vector<float> target, float distanceTol, std::vector<int> &ids, uint depth)
	{

		// Define target box boundaries
		float x1 = target[0] - distanceTol;
		float x2 = target[0] + distanceTol;
		float y1 = target[1] - distanceTol;
		float y2 = target[1] + distanceTol;

		if (node != NULL)
		{
			// Check if the node point is within target box
			if (node->point[0] >= x1 && node->point[0] <= x2 && node->point[1] >= y1 && node->point[1] <= y2)
			{
				// Calculate euclidean distance of target from the node point
				float distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2));
				// If the node point is within the distanceTol add it in ids
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			// Check if the target box crosses left node boundary
			if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
				// Explore the left subtree
				searchRecursive(node->left, target, distanceTol, ids, depth + 1);

			// Check if the target box crosses right node boundary
			if ((target[depth % 2] + distanceTol) > node->point[depth % 2])
				// Explore the right subtree
				searchRecursive(node->right, target, distanceTol, ids, depth + 1);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// Recursive helper function to search for all nearest neighbors of target
		searchRecursive(root, target, distanceTol, ids, 0);

		return ids;
	}
};
