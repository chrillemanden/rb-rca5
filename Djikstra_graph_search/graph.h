#ifndef GRAPH_H
#define GRAPH_H
#include "node.h"
#include <opencv2/opencv.hpp>
#include <vector>

struct CmpNodePtrs
{
    bool operator()(const Node* lhs, const Node* rhs) const
    {
        return lhs->dist_to_start > rhs->dist_to_start;
    }
};


class Graph
{

public:
    Graph(cv::Mat map, std::vector<cv::Point2i> points);
    std::vector<Node*> djikstra_search(int start_x, int start_y, int end_x, int end_y);

private:
    int get_node_index(int x, int y);
    void create_adjancecy_list(cv::Mat map, std::vector<cv::Point2i> points);
    void create_node_list( std::vector<cv::Point2i> points);
    void reset_node_list();
    double dist_between_nodes(Node* current_node, Node* next_node);

    std::vector<std::vector<Node*>> adjacency_list;
    std::vector<Node> node_list;
};

#endif // GRAPH_H
