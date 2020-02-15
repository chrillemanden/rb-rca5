#ifndef EST_H
#define EST_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <random>
#include <stdlib.h>
#include <cstdlib>




struct vertex {
    int x_pos;
    int y_pos;
    int n_conn = 0;
    double weight = 0.0;
    //vertex from_v = nullptr;
    int origin_indx = -1;
};






/*
    Class for building an Expansive Space Tree (EST) on a 2D map
    EST objects are used when performing an EST-query to get from q_start to q_goal on a map
*/
class EST
{
public:
    EST();
    EST(vertex q_start, cv::Mat map);

    vertex extendEST(cv::Mat &map_show);
    vertex getClosest(vertex q_new);

    std::vector<vertex> vertices;

private:
    int number_edges = 0;
    int h;
    int w;
    cv::Mat map;

};

/*
 * Function that returns true if there exists a collision-free path between q1 and q2
 * in the provided 2D map
 */
bool collFreePath(vertex q1, vertex q2, cv::Mat map);


/*
 * Function that given a list of vertices and a 2d-map, finds shorter collision free paths
 * among the vertices and returns a list of these. Used for post-processing of generated
 * single-query paths
 *
 */
void shortenPath(std::vector<vertex> vertices, cv::Mat map);


/*
 * Function that given two trees and two configurations, one from each tree, where a collision
 * free path exists beteem the configurations, returns a list of vertices that contains a path
 * from root of one tree to the root of the other.
 */
void isolatePath(EST start_tree, EST goal_tree, vertex q1, vertex q2, cv::Mat map);


/*
 * Function that given two configurations on a 2d map, finds a path between the configurations
 * by growing Expansive Space Trees (EST) from each configuration
 */
void ESTquery(vertex q_start, vertex q_goal, int n, cv::Mat map);




#endif // EST_H
