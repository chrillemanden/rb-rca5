#ifndef NODE_H
#define NODE_H


class Node
{

public:
    Node(int x, int y, int index_num);
    void update_connection(Node* new_connection);
    void update_distance(double new_distance);
    bool is_pos(int end_x, int end_y);
    double get_dist_to_start();

    int x_coordinate;
    int y_coordinate;
    Node* connected_via;
    double dist_to_start;
    int index;

    ~Node();
};

#endif // NODE_H
