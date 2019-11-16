#include "node.h"
#include <cfloat>

Node::Node(int x, int y, int index_num)
{
    x_coordinate = x;
    y_coordinate = y;
    connected_via = nullptr;
    dist_to_start = DBL_MAX;
    //expanded = false;
    index = index_num;
}



void Node::update_connection(Node *new_connection)
{
    connected_via = new_connection;
}

void Node::update_distance(double new_distance)
{
    dist_to_start = new_distance;
}

bool Node::is_pos(int end_x, int end_y)
{
    if(x_coordinate == end_x)
    {
        if(y_coordinate == end_y)
        {
            return true;
        }
    }
    return false;
}

double Node::get_dist_to_start()
{
    return dist_to_start;
}


Node::~Node()
{

}
