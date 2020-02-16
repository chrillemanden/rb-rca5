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

static std::random_device rde;
static std::default_random_engine generat(rde());


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
    //q_goal.x_pos = 800;
    //q_goal.y_pos = 360;

    q_goal.x_pos = 30;
    q_goal.y_pos = 30;

//    // Make a query from one configuration to the other on the map
//    std::vector<vertex> path = ESTquery(q_start,q_goal, 200, map);

//    // Print all the configurations in the path
//    for (auto &v : path)
//    {
//        std::cout << "x: " << v.x_pos << " , y: " << v.y_pos << std::endl;
//    }

//    RRT rrt_demo(q_start, map);
//    for (int i = 0; i < 2000; i++)
//    {
//        // Generate a random collision free vertex on the map
//        std::uniform_int_distribution<int> y(0, map.rows);
//        std::uniform_int_distribution<int> x(0, map.cols);
//        vertex q;
//        q.x_pos = x(generat);
//        q.y_pos = y(generat);

//        rrt_demo.extendRRT(map, q);
//    }

    RRTquery(q_start, q_goal, 1000, map);



    std::cout << "Demonstration done!" << std::endl;
    cv::waitKey(0);
    return 0;
}


