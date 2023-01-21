#include <iostream>
#include <deque>  // For frontier
#include <vector> // For maze
#include <set>    // For visited nodes
#include <map>    // Mapping each node to their shortest path

using std::deque;
using std::map;
using std::set;
using std::vector;

// Returns all the neighbors of the given node within the boundary
vector<vector<int>> find_neighbors(vector<int> &node, int nRows, int nCols);
// Prints the path associated with the pathKey
void printPath(map<vector<int>, vector<vector<int>>> &path, vector<int> &pathKey);
// Iterate over vector of vectors and for each of the
// nested vector print its contents
template <typename T>
void print_2d_vector(const vector<vector<T>> &matrix);

int main()
{
    vector<vector<char>> maze{
        {'#', '#', 'E', '#', '.'},
        {'.', '.', '.', '#', '#'},
        {'#', '.', '#', '#', '.'},
        {'#', '.', '.', 'S', '.'}};
    deque<vector<vector<int>>> frontier;
    set<vector<int>> visited;
    vector<vector<int>> neighbors;
    vector<vector<int>> startNode, currNode, tempVect;
    // key -> node coordinate, value-> shortest path from start to this node
    // The path includes all the node coordinates as vector of vector
    map<vector<int>, vector<vector<int>>> path;

    // Find the start point and assign coordinates, distance and path to the start node
    for (int i = 0; i < maze.size(); ++i)
    {
        for (int j = 0; j < maze[i].size(); ++j)
        {
            if (maze[i][j] == 'S')
            {
                path[{i, j}] = {{i, j}};
                startNode = {{i, j}, {0}};
            }
        }
    }

    // Add start node to frontier
    frontier.push_back(startNode);

    bool exitFound = false;

    // Loop through the frontier
    // extract the first node from frontier and find its neighbors
    while (!frontier.empty())
    {
        currNode = frontier.front();
        frontier.pop_front(); // delete the front node from frontier

        // End the search of the current extracted node is exit('E') and print result
        if (maze[currNode[0][0]][currNode[0][1]] == 'E')
        {
            exitFound = true;
            std::cout << "Exit Found!" << std::endl;
            std::cout << "Coordinates are: "
                      << "{" << currNode[0][0] << ","
                      << " " << currNode[0][1] << "}" << std::endl;
            std::cout << "Shortest distance to exit is: " << currNode[1][0] << std::endl;
            std::cout << "The shortest path (start -> exit) is: " << std::endl;
            printPath(path, currNode[0]);
        }

        neighbors = find_neighbors(currNode[0], maze.size() - 1, maze[0].size() - 1);

        // Iterate through all nodes in the neighbor
        // If the neighbor is not a wall ('#') or already visited then add it in frontier
        // Calculate the path to each neighbor node from its parent
        // Note: The parent node will already contain the path from start ('S')
        for (auto &node : neighbors)
        {
            char nodeVal = maze[node[0]][node[1]];
            if (visited.find(node) != visited.end() || nodeVal == '#')
            {
                continue;
            }
            else
            {
                tempVect = {{node[0], node[1]}};
                for (auto &node : path[currNode[0]])
                {
                    tempVect.push_back(node);
                }

                path[{node[0], node[1]}] = tempVect;

                frontier.push_back({{node[0], node[1]}, {currNode[1][0] + 1}});
            }
        }

        visited.insert({currNode[0][0], currNode[0][1]});
    }

    if (!exitFound)
    {
        std::cout << "No exit found!!" << std::endl;
    }
    return 0;
}

vector<vector<int>> find_neighbors(vector<int> &node, int nRows, int nCols)
{
    vector<vector<int>> neighbors;
    vector<int> rowMov = {-1, 1, 0, 0};
    vector<int> colMov = {0, 0, -1, 1};
    int rr, cc;

    for (int i = 0; i < rowMov.size(); ++i)
    {
        rr = rowMov[i] + node[0];
        cc = colMov[i] + node[1];
        if (rr > nRows || cc > nCols || rr < 0 || cc < 0)
        {
            continue;
        }
        neighbors.push_back({rr, cc});
    }

    return neighbors;
}

// Iterate over vector of vectors and for each of the
// nested vector print its contents
template <typename T>
void print_2d_vector(const vector<vector<T>> &matrix)
{
    for (auto row_obj : matrix)
    {
        for (auto elem : row_obj)
        {
            std::cout << elem << ", ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void printPath(map<vector<int>, vector<vector<int>>> &path, vector<int> &pathKey)
{
    vector<vector<int>> outPath;
    outPath = path[pathKey];

    for (auto &node : outPath)
    {
        std::cout << "{" << node[0] << ", " << node[1] << "}"
                  << " <- ";
    }
    std::cout << std::endl;
}