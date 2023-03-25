/* \author Aaron Brown */
// Nikhil Nair: Implement insert and search function which works for 2-d and 3-dimensional points
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

		// Get the dimensions of point
		uint dim = point.size();

		// Limit dim to max 3 dimensions (x, y, z)
		if (dim > 3)
		{
			dim = 3;
		}

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
				// Alternate between each dim of point and insert the point in the empty slot
				if (point[depth % dim] < temp->point[depth % dim])
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
				// If there are no empty slots in current level go to next depth (level) of tree
				depth += 1;
			}
		}

		return;
	}

	/**
	 * @brief Recursive helper function to search for all nearest neighbors of target
	 *
	 * @param node Current node in tree
	 * @param target Target point to search for
	 * @param distanceTol nearest neighbor distance tolerance
	 * @param ids list of point ids in the tree that are within distance tol of target
	 * @param depth Depth of k-d tree
	 */
	void searchRecursive(Node *node, std::vector<float> target, float distanceTol, std::vector<int> &ids, uint depth)
	{

		// Get the dimensions of target point
		uint dim = target.size();

		// Limit dim to max 3 dimensions (x, y, z)
		if (dim > 3)
		{
			dim = 3;
		}

		// Define target box boundaries
		// Note: Only spatial dimensions are used for the search criteria. This will work for both 2-d and 3-d cases

		// Define an empty array of size 2*dim (size of target box)
		std::vector<float> tarBox(2 * dim);

		// Define the boundaries of the target box
		for (uint i = 0; i < dim; ++i)
		{
			tarBox[i] = target[i] - distanceTol;
			tarBox[i + dim] = target[i] + distanceTol;
		}

		if (node != NULL)
		{
			bool isWithin = true;
			// Check if the node point is within target box tarBox.
			for (uint j = 0; j < dim; ++j)
			{
				if (node->point[j] < tarBox[j] || node->point[j] > tarBox[j + dim])
				{
					isWithin = false;
					break;
				}
			}

			if (isWithin)
			{
				// Calculate euclidean distance of target from the node point
				float distance = 0.0;
				for (uint k = 0; k < dim; ++k)
				{
					distance += pow(node->point[k] - target[k], 2);
				}
				distance = sqrt(distance);

				// If the node point is within the distanceTol add it in ids
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			// Check if the target box crosses left node boundary
			if ((target[depth % dim] - distanceTol) < node->point[depth % dim])
				// Explore the left subtree
				searchRecursive(node->left, target, distanceTol, ids, depth + 1);

			// Check if the target box crosses right node boundary
			if ((target[depth % dim] + distanceTol) > node->point[depth % dim])
				// Explore the right subtree
				searchRecursive(node->right, target, distanceTol, ids, depth + 1);
		}
	}

	/**
	 * @brief Search for all nearest neighbors of target
	 *
	 * @param target Target point to search for
	 * @param distanceTol nearest neighbor distance tolerance
	 * @return std::vector<int> list of point ids in the tree that are within distance tol of target
	 */
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// Recursive helper function to search for all nearest neighbors of target
		searchRecursive(root, target, distanceTol, ids, 0);

		return ids;
	}
};
