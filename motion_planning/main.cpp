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





// algorithm for building the Expansive Space Tree
//void buildEST(vertex q_start, int n, cv::Mat &map)
//{
//    // Show the first point
//    cv::Mat map_show = map.clone();
//    cv::circle(map_show, cv::Point2i(q_start.x_pos,q_start.y_pos), 4, cv::Scalar(0,128,0), 2, 8);

//    // Initialise lists for storing vertices and edges
//    std::vector<vertex> vertices;
//    vertices.push_back(q_start);
//    std::vector<edge> edges;

//    // Build the random generators for generating new vertices
//    std::uniform_int_distribution<int> move_y(-100, 100);
//    std::uniform_int_distribution<int> move_x(-100, 100);
//    //std::normal_distribution<int> move_y(10.0, 2.0);
//    //std::normal_distribution<int> move_x(10.0, 2.0);

//    int number_edges = 0;

//    for (int i = 1; i <= n; i++)
//    {

//        // Sample a random vertex from the vertex vector with a higher probability for choosing unexplored vertices
//        int q_index = pi_t(vertices);
//        vertex q = vertices[q_index];


//        // Extend the tree
//        // find qnew close to q
//        vertex q_new;
//        q_new.x_pos = q.x_pos + move_x(generator);
//        q_new.y_pos = q.y_pos + move_y(generator);


//        /* Check if there is a collision free path from q to qnew */

//        //Walk along the length of the line between the two points
//        cv::LineIterator line_it(map, cv::Point2i(q_new.x_pos, q_new.y_pos), cv::Point2i(q.x_pos, q.y_pos), 8);

//        for(int k = 0; k < line_it.count; line_it++, k++)
//        {
//            cv::Vec3b pixel_val = map.at<cv::Vec3b>(line_it.pos());
//            //std::cout << "pixel_val" << pixel_val << std::endl;

//            //check if a wall pixel is encountered
//            if(pixel_val == cv::Vec3b(0,0,0))
//            {
//                //std::cout << "I: "<< k << "black pixel found" << std::endl;
//                break;
//            }

//            //store the connection if no wall pixels is encountered along the length of line
//            if(k + 1 == line_it.count)
//            {
//                number_edges++;
//                q_new.n_conn += 1;
//                //q_new.weight = ((double)vertices.size() + 1.0) / (double)q_new.n_conn;
//                q_new.weight = (double)number_edges / (double)q_new.n_conn;
//                //q_new.conn.push_back(*vertices[q_index]);
//                vertices.push_back(q_new);
//                //vertices[q_index].conn.push_back(*vertices[vertices.size()-1]);
//                vertices[q_index].n_conn += 1;
//                //vertices[q_index].weight = (double)vertices.size() / (double)vertices[q_index].n_conn;
//                vertices[q_index].weight = (double)number_edges / (double)vertices[q_index].n_conn;

//                cv::circle(map_show, cv::Point2i(q_new.x_pos,q_new.y_pos), 4, cv::Scalar(0,0,255), 2, 8);
//                cv::line(map_show, cv::Point2i(q_new.x_pos,q_new.y_pos), cv::Point2i(q.x_pos,q.y_pos), cv::Scalar(0,127,127), 2);

//                break;
//            }
//        }

//        showImage("EST", map_show);

//        cv::waitKey(1);

//    }
//}

int main()
{
    srand (time(NULL));

    // Load the map and resize to a bigger size
    cv::Mat map = cv::imread("floor_plan.png");
    //showImage("initial", map);
    int map_scale = 10;
    cv::resize(map, map, cv::Size(), map_scale, map_scale, cv::INTER_AREA);
    cv::Mat map_show = map.clone();

    // Good starting position
    vertex q_start;
    q_start.x_pos = 800;
    q_start.y_pos = 760;

    // Good ending position
    vertex q_goal;
    q_goal.x_pos = 800;
    q_goal.y_pos = 360;

    //buildEST(q_start, 10000, map);


    ESTquery(q_start,q_goal, 200, map);


    cv::waitKey(0);


    std::cout << "Demonstration done!" << std::endl;
    return 0;
}


