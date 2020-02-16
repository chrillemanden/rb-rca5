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

    /*
     * Extend the tree with a new configuration
     * return the new configuration
     */
    vertex extendEST(cv::Mat map_show);

    /*
     * Given a configuration, return the configuration in the tree closest to
     */
    vertex getClosest(vertex q_new);

    std::vector<vertex> vertices;

private:
    int number_edges = 0;
    int h;
    int w;
    cv::Mat map;

};

/*
    Class for building an Rapidly Exploring Random Trees (RTT) on a 2D map
    RRT objects are used when performing an RRT-query to get from q_start to q_goal on a map
*/
class RRT
{
public:
    RRT();
    RRT(vertex q_start, cv::Mat map);

    /*
     * Extend the tree with a new configuration
     * return the new configuration
     */
    vertex extendRRT(cv::Mat &map_show, vertex q);

    /*
     * Given a configuration, return the configuration in the tree closest to
     */
    vertex getClosest(vertex q_new);

    std::vector<vertex> vertices;

private:
    int number_edges = 0;
    int h;
    int w;
    int step_size;
    int q_indx;
    cv::Mat map;

};

/*
 * Function that given two vertices in a 2d map, returns the distance between them
 */
double vDist(vertex q1, vertex q2);

/*
 * Function that returns true if there exists a collision-free path between q1 and q2
 * in the provided 2D map
 */
bool collFreePath(vertex q1, vertex q2, cv::Mat map);


/*
 * Function that given a list of vertices and a 2d-map, finds shorter collision free paths
 * among the vertices and returns a list of these. Used for post-processing of generated
 * single-query paths
 * The new path is returned in the same vector that is input
 */
void shortenPath(std::vector<vertex> &vertices, cv::Mat map);


/*
 * Function that given two trees and two configurations, one from each tree, where a collision
 * free path exists beteem the configurations, returns a list of vertices that contains a path
 * from root of one tree to the root of the other.
 */
template <typename Tree>
std::vector<vertex> isolatePath(Tree start_tree, Tree goal_tree, vertex q1, vertex q2, cv::Mat map);


/*
 * Function that given two configurations on a 2d map, finds a path between the configurations
 * by growing Expansive Space Trees (EST) from each configuration
 * Returns the path in vector of vertices
 */
std::vector<vertex> ESTquery(vertex q_start, vertex q_goal, int n, cv::Mat map);



std::vector<vertex> RRTquery(vertex q_start, vertex q_goal, int n, cv::Mat map);

/*
 * Function that builds a single Expansive Spaces Tree in a 2d map
 */
void buildSingleEST(vertex q_start, int n, cv::Mat &map);


#endif // EST_H
