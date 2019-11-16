#include "graph.h"
#include <iostream>
#include <vector>
#include <queue>


Graph::Graph(cv::Mat map, std::vector<cv::Point2i> points)
{
    create_node_list(points);
    create_adjancecy_list(map, points);
}

std::vector<Node *> Graph::djikstra_search(int start_x, int start_y, int end_x, int end_y)
{
    std::priority_queue <Node*, std::vector<Node*>, CmpNodePtrs> node_PQ;

    int start_index = get_node_index(start_x, start_y);


    node_list[start_index].update_distance(0);
    node_PQ.push(&node_list[start_index]);

   while(!node_PQ.empty())
   {
       // Get the node with smallest distance to the start node, which has not been expanded previous.
       Node* cur_node = node_PQ.top();
       // Remove the node from the PQ
       node_PQ.pop();

       //If the next node to expand is the end node. Then a better route do not exist.
       if(cur_node->is_pos(end_x, end_y))
       {
           break;
       }

       // List through the nodes connections to
       for(unsigned int i = 0; i <adjacency_list[cur_node->index].size(); i++)
       {
           Node* test_node = adjacency_list[cur_node->index][i];
           //Check if the distance from start is smaller than the previous recorded distance
           double distance = dist_between_nodes(cur_node, test_node);

           if(cur_node->get_dist_to_start() + distance < test_node->get_dist_to_start())
           {
               //Update distance and connection on the test node
               test_node->update_distance(cur_node->get_dist_to_start()+distance);
               test_node->update_connection(cur_node);

               //Push node into PQ.
               node_PQ.push(test_node);
           }

       }

   }

   int end_index = get_node_index(end_x, end_y);

   Node* node = &node_list[end_index];
   std::vector<Node *> output_list;

   while(node->is_pos(start_x, start_y) == false)
   {
       output_list.push_back(node);
       node = node->connected_via;
   }

   output_list.push_back(node);

   reset_node_list();
   return output_list;
}

void Graph::create_adjancecy_list(cv::Mat map, std::vector<cv::Point2i> points)
{
    std::vector<Node*> vec = {};
    std::vector<std::vector<Node*>> output_vec(points.size(), vec);

    for(unsigned int i = 0; i < points.size(); i++)
    {
        for(unsigned int j = 0; j < points.size(); j++)
        {
            //Prevent creating a direct loop back to one point.
            if(j == i)
            {
                continue;
            }

            double line_length = sqrt(pow((double) points[j].x-points[i].x, 2.0)+pow((double) points[j].y-points[i].y, 2.0)); //Calculate length of c.

            //Discard a connection if it is to long.
            if(line_length > map.rows * 0.5 || line_length > map.cols * 0.5)
            {
                continue;
            }

            //Walk along the length of the line between the two points
            cv::LineIterator line_it(map, points[i], points[j], 8);

            for(int k = 0; k < line_it.count; line_it++, k++)
            {
                cv::Vec3b pixel_val = map.at<cv::Vec3b>(line_it.pos());
                //std::cout << "pixel_val" << pixel_val << std::endl;

                //check if a wall pixel is encountered
                if(pixel_val == cv::Vec3b(0,0,0))
                {
                    //std::cout << "I: "<< k << "black pixel found" << std::endl;
                    break;
                }

                //store the connection if no wall pixels is encountered along the length of line
                if(k + 1 == line_it.count)
                {
                    output_vec[i].push_back(&node_list[j]);
                    break;
                }
            }

        }
    }

    adjacency_list = output_vec;

}

void Graph::create_node_list(std::vector<cv::Point2i> points)
{
    node_list.clear();

    for(unsigned int i = 0; i < points.size(); i++)
    {
        node_list.push_back(Node(points[i].x, points[i].y, i));
    }
}

void Graph::reset_node_list()
{
    for(unsigned int i = 0; i < node_list.size(); i++)
    {
        node_list[i].dist_to_start = DBL_MAX;
        node_list[i].connected_via = nullptr;
    }
}

int Graph::get_node_index(int x, int y)
{
    for(unsigned int i = 0; i < node_list.size(); i++)
    {
        if(node_list[i].x_coordinate == x)
        {
            if(node_list[i].y_coordinate ==  y)
            {
                return i;
            }
        }

    }

    std::cout << "Not a valid coordinate" << std::endl;
    return -1;
}

double Graph::dist_between_nodes(Node *current_node, Node *next_node)
{
    double x_cur = current_node->x_coordinate;
    double y_cur = current_node->y_coordinate;

    double x_next = next_node->x_coordinate;
    double y_next = next_node->y_coordinate;

    return sqrt(pow(x_cur-x_next, 2.0) + pow(y_cur-y_next, 2.0));
}


