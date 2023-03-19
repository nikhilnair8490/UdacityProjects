/* \author Aaron Brown */
// Nikhil Nair: Implement insert function
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		// Function to insert a new point into the tree

		//Insert the point as root if the root is NULL
		if (root == NULL)
        {
            root = new Node(point, id);
            return;
        }
        else
        {
            // Create a new node from the point
			Node* newNode = new Node(point, id);
			//Traverse through the tree and find an empty slot to place the new point
			Node* temp = root; //Start from root node
			uint depth = 0; // depth of tree
            while (temp!= NULL)
            {
                if (depth%2 ==0 ) // tree depth for x split
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
				//If there are no empty slots in current level go to next depth level of tree
				depth += 1;

            }
        }

		return;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




