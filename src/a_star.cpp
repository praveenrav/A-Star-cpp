#include <iostream>
#include <vector>
// #include <cstdlib>
#include <ctime>
#include <cmath>
#include <queue>
#include <unordered_set>

#include "opencv2/opencv.hpp"
#include "a_star.hpp"


A_Star::A_Star(int numRows, int numColumns) : numRows_(numRows), numColumns_(numColumns)
{
    // Initialize the random number generator
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    // Initialize the grid based on user-inputted dimensions using vectors:
    grid_.resize(numColumns_, std::vector<Node>(numRows_, Node{false, 0, 0, -1, -1, FLT_MAX, FLT_MAX, FLT_MAX}));

    // Initialize the grid based on user-inputted dimensions:
    for (int x = 0; x < numColumns_; x++) {
        for (int y = 0; y < numRows_; y++) {
            grid_[x][y] = Node{false, x, y, -1, -1, FLT_MAX, FLT_MAX, FLT_MAX};            
        }
    }    while(!openSet_queue.empty())
    {
        // Get the node with the lowest fCost from the open set
        Node curr = openSet_queue.top();
        openSet_queue.pop();

        // Display the image in real-time
        drawGrid();

        // Check if the current node is the goal node, and obtain stack containing path if true:
        if (curr.x == goalPos.x && curr.y == goalPos.y) 
        {
            reachedGoal = true;
            obtainPathStack();

            while(!pathStack.empty())
            {
                pathSet.insert(pathStack.top());
                pathStack.pop();

                // Display the image in real-time
                drawGrid();
            }

            break;
        }


        // Generating neighboring nodes (up, down, left, right):
        std::vector<std::pair<int, int>> neighbors = 
        {
            {curr.x - 1, curr.y}, {curr.x + 1, curr.y},
            {curr.x, curr.y - 1}, {curr.x, curr.y + 1},
        };


        // Examining each potential neighbor:
        for(const auto& neighbor : neighbors) 
        {
            int neighborX = neighbor.first;
            int neighborY = neighbor.second;
            float tentativeGCost;

            if(isValidCell(neighborX, neighborY) && !isOccupied(neighborX, neighborY))
            {
                tentativeGCost = curr.gCost + getNodeDist(curr, grid_[neighborX][neighborY]);
            }
            else
            {
                continue; // Skip this neighbor
            }

            // Check if neighbor is already in closed set:
            if(closedSet.find({neighborX, neighborY}) != closedSet.end()) 
            {
                continue; // Skip this neighbor
            }


            if (tentativeGCost < grid_[neighborX][neighborY].gCost)
            {
                // Update the neighbor's parameters:
                grid_[neighborX][neighborY].parentX = curr.x;
                grid_[neighborX][neighborY].parentY = curr.y;
                grid_[neighborX][neighborY].gCost = tentativeGCost;
                grid_[neighborX][neighborY].hCost = getNodeDist(grid_[neighborX][neighborY], goalPos);
                grid_[neighborX][neighborY].fCost = grid_[neighborX][neighborY].gCost + grid_[neighborX][neighborY].hCost;
                
                openSet.insert({neighborX, neighborY});
                openSet_queue.push(grid_[neighborX][neighborY]);
                
                drawGrid();
            }
        }

        closedSet.insert({curr.x, curr.y});

        drawGrid();

    }

    if(!reachedGoal)
    {
        std::cout << "Failed to reach goal." << std::endl;
    }

    // Create the OpenCV Image and initialize with a white background:
    occupancyGridImage_ = cv::Mat(numRows_, numColumns_, CV_8UC3, cv::Scalar(255, 255, 255));


}



void A_Star::runAStar()
{
            
    // Run A* method:
    openSet_queue.push(startPos);
    drawGrid();

    while(!openSet_queue.empty())
    {
        // Get the node with the lowest fCost from the open set
        Node curr = openSet_queue.top();
        openSet_queue.pop();

        // Display the image in real-time
        drawGrid();

        // Check if the current node is the goal node, and obtain stack containing path if true:
        if (curr.x == goalPos.x && curr.y == goalPos.y) 
        {
            reachedGoal = true;
            obtainPathStack();

            while(!pathStack.empty())
            {
                pathSet.insert(pathStack.top());
                pathStack.pop();

                // Display the image in real-time
                drawGrid();
            }

            break;
        }


        // Generating neighboring nodes (up, down, left, right):
        std::vector<std::pair<int, int>> neighbors = 
        {
            {curr.x - 1, curr.y}, {curr.x + 1, curr.y},
            {curr.x, curr.y - 1}, {curr.x, curr.y + 1},
        };


        // Examining each potential neighbor:
        for(const auto& neighbor : neighbors) 
        {
            int neighborX = neighbor.first;
            int neighborY = neighbor.second;
            float tentativeGCost;

            if(isValidCell(neighborX, neighborY) && !isOccupied(neighborX, neighborY))
            {
                tentativeGCost = curr.gCost + getNodeDist(curr, grid_[neighborX][neighborY]);
            }
            else
            {
                continue; // Skip this neighbor
            }

            // Check if neighbor is already in closed set:
            if(closedSet.find({neighborX, neighborY}) != closedSet.end()) 
            {
                continue; // Skip this neighbor
            }


            if (tentativeGCost < grid_[neighborX][neighborY].gCost)
            {
                // Update the neighbor's parameters:
                grid_[neighborX][neighborY].parentX = curr.x;
                grid_[neighborX][neighborY].parentY = curr.y;
                grid_[neighborX][neighborY].gCost = tentativeGCost;
                grid_[neighborX][neighborY].hCost = getNodeDist(grid_[neighborX][neighborY], goalPos);
                grid_[neighborX][neighborY].fCost = grid_[neighborX][neighborY].gCost + grid_[neighborX][neighborY].hCost;
                
                openSet.insert({neighborX, neighborY});
                openSet_queue.push(grid_[neighborX][neighborY]);
                
                drawGrid();
            }
        }

        closedSet.insert({curr.x, curr.y});

        drawGrid();

    }

    if(!reachedGoal)
    {
        std::cout << "Failed to reach goal." << std::endl;
    }

    cv::destroyAllWindows();

}



void A_Star::obtainPathStack()
{
    Node curr = goalPos;

    while(!(curr.x == startPos.x && curr.y == goalPos.y))
    {
        pathStack.push({curr.x, curr.y});
        curr = grid_[curr.parentX][curr.parentY];
    }

}


// Method to randomly fill grid with occupied cells:
void A_Star::randomFillGrid(float occupancyProbability)
{
        
    for (int x = 0; x < numRows_; x++) {
        for (int y = 0; y < numColumns_; y++) {
            
            float randomValue = static_cast<float>(std::rand()) / RAND_MAX; // Generate a random float between 0 and 1
            
            if (randomValue < occupancyProbability) {
                setOccupied(x, y); // Occupy cell
            }
        }
    }

}


// Method to set the inputted node to occupied:
void A_Star::setOccupied(int x, int y)
{
    if(isValidCell(x, y))
    {
        grid_[x][y].isOccupied = true;
    }
}


// Method to determine whether or not the inputted node is occupied:
bool A_Star::isOccupied(int x, int y)
{
    if(isValidCell(x, y))
    {
        return grid_[x][y].isOccupied;
    }

    return false;
}


// Function to calculate distance between two nodes:
float A_Star::getNodeDist(Node node1, Node node2)
{
    return sqrt( ((node1.x - node2.x) * (node1.x - node2.x)) + 
    ((node1.y - node2.y) * (node1.y - node2.y)) );

}


// Method to return whether or not the given x- and y-coordinates correspond to a valid cell:
bool A_Star::isValidCell(int x, int y) 
{
    return (x >= 0 && x < numColumns_ && y >= 0 && y < numRows_);
}


// Function to set the start and goal poses of the algorithm:
void A_Star::setStartGoalPoses(int start_x, int start_y, int goal_x, int goal_y)
{

    // Setting x- and y-coordinates for start and goal nodes:
    startPos.isOccupied = false;
    startPos.x = start_x;
    startPos.y = start_y;
    
    goalPos.isOccupied = false;
    goalPos.x = goal_x;
    goalPos.y = goal_y;


    // Updating costs:
    startPos.gCost = 0.0f;
    startPos.hCost = getNodeDist(startPos, goalPos);
    startPos.fCost = startPos.fCost; 

    goalPos.gCost = getNodeDist(startPos, goalPos);
    goalPos.hCost = 0.0f;
    goalPos.fCost = goalPos.gCost; 
}



void A_Star::drawGrid()
{
    
    // Image parameters:
    int cellSize = 20; // Size of cell
    cv::Scalar borderColor(0, 0, 0); // Black border color
    int borderThickness = 2; // Border thickness in pixels

    // Loop through the grid and draw cells
    for (int x = 0; x < numRows_; x++) 
    {
        for (int y = 0; y < numColumns_; y++) 
        {
            cv::Rect cell(y * cellSize, x * cellSize, cellSize, cellSize);
            
            // Set cell color based on conditions
            cv::Scalar color;
            if (x == startPos.y && y == startPos.x) {
                color = cv::Scalar(0, 0, 255); // Red
            } else if (x == goalPos.y && y == goalPos.x) {
                color = cv::Scalar(255, 0, 0); // Green
            } else if (openSet.find({y, x}) != openSet.end()) {
                color = cv::Scalar(0, 165, 255); // Orange
            } else if (closedSet.find({y, x}) != closedSet.end()) {
                color = cv::Scalar(255, 0, 255); // Purple
            } else if (pathSet.find({y, x}) != pathSet.end()) {
                color = cv::Scalar(0, 255, 0); // Blue
            } else if (!grid_[x][y].isOccupied) {
                color = cv::Scalar(255, 255, 255); // White
            } else {
                color = cv::Scalar(0, 0, 0); // Black
            }

            // Fill the cell with the determined color and adding border:
            cv::rectangle(occupancyGridImage_, cell, color, -1); // Filling in rectangle
            cv::rectangle(occupancyGridImage_, cell, borderColor, borderThickness); // Adding border
        }
    }

    // Show the OpenCV image in a window
    cv::imshow("A* Visualization", occupancyGridImage_);

    cv::waitKey(10000); // Wait until a key is pressed to close the window
}
