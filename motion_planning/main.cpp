//#include <gazebo/gazebo_client.hh>
//#include <gazebo/msgs/msgs.hh>
//#include <gazebo/transport/transport.hh>
//#include "fl/Headers.h"
#include <boost/math/constants/constants.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>
#include <fstream>
#include <stdlib.h>
#include <cstdlib>

/* Class includes */
#include "est.h"

/* Driver includes */


int main()
{
    // Seed the random generator
    srand (time(NULL));

    // Load the map and resize to a bigger size
    cv::Mat map = cv::imread("floor_plan.png");
    int map_scale = 10;
    cv::resize(map, map, cv::Size(), map_scale, map_scale, cv::INTER_AREA);


    // Define starting position
    vertex q_start;
    q_start.x_pos = 800;
    q_start.y_pos = 760;

    // Define ending position
    vertex q_goal;
    q_goal.x_pos = 800;
    q_goal.y_pos = 360;

//    // Make a query from one configuration to the other on the map
//    std::vector<vertex> path = ESTquery(q_start,q_goal, 200, map);

//    // Print all the configurations in the path
//    for (auto &v : path)
//    {
//        std::cout << "x: " << v.x_pos << " , y: " << v.y_pos << std::endl;
//    }

    RRT rrt_demo(q_start, map);
    for (int i = 0; i < 5; i++)
    {
        rrt_demo.extendRRT(map);
    }



    std::cout << "Demonstration done!" << std::endl;
    cv::waitKey(0);
    return 0;
}


