#ifndef A_STAR
#define A_STAR

#include <iostream>
#include <unordered_set>
#include <utility>
#include <queue>
#include <stack>

#include "opencv2/opencv.hpp"



// Custom structure to represent coordinates with priority:
struct Node 
{
    bool isOccupied;
    int x;
    int y;
    int parentX;
    int parentY;
    float gCost;
    float hCost; 
    float fCost;
};


// Custom comparison function for the priority queue:
struct CompareCoordinates {
    
    bool operator()(const Node& lhs, const Node& rhs) const {                
        
        // Compare elements based on their priority values (lower values have higher priority)
        return lhs.fCost < rhs.fCost;
    }

};


// Custom hash function for pairs of integers
struct PairHash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ h2; // Combine the hash values
    }
};

class A_Star {


    public:
        A_Star(int numRows, int numColumns);

        int numRows_;
        int numColumns_;
        std::vector<std::vector<Node>> grid_;

        void runAStar();
        void drawGrid();
        void randomFillGrid(float occupancyProbability);
        void setStartGoalPoses(int startPos_x, int startPos_y, int goalPose_x, int goalPose_y);
        

    private:

        void obtainPathStack();
        void setOccupied(int x, int y);
        bool isOccupied(int x, int y);
        bool isValidCell(int x, int y);
        float getNodeDist(Node node1, Node node2);

        Node startPos = Node{false, -1, -1, -1, -1, FLT_MAX, FLT_MAX, FLT_MAX};
        Node goalPos = Node{false, -1, -1, -1, -1, FLT_MAX, FLT_MAX, FLT_MAX};

        bool reachedGoal = false;

        int delay_graph = 100;

        cv::Mat occupancyGridImage_;

        // Priority queue to store all open nodes:
        std::priority_queue<Node, std::vector<Node>, CompareCoordinates> openSet_queue;

        // Set to store all open nodes:
        std::unordered_set<std::pair<int, int>, PairHash> openSet;

        // Set to store all visited nodes:
        std::unordered_set<std::pair<int, int>, PairHash> closedSet;

        // Stack to eventually store final path from start position to goal position:
        std::stack<std::pair<int, int>> pathStack;

        // Set to store all nodes within the final path:
        std::unordered_set<std::pair<int, int>, PairHash> pathSet;

};



#endif


