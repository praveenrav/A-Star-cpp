#include <iostream>
#include "a_star.hpp"
#include "opencv2/opencv.hpp"

int main() {

    // Create an instance of the A_Star class:
    A_Star a_star(15, 12); // Assuming a 20x20 grid
    a_star.randomFillGrid(0.2); // Randomly filling 20% of the grid

    // Set the start and goal positions:
    a_star.setStartGoalPoses(0, 0, 2, 3);

    // Running A* algorithm, displaying it visually via OpenCV:
    a_star.runAStar();

    return 0;
}