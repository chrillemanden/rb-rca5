#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "map_util.h"
#include "node.h"
#include "graph.h"


using namespace std;

vector<cv::Point2i> switch_XY_coordinates(vector<cv::Point2i> list)
{
    vector<cv::Point2i> output_vec = {};

    for(unsigned int i = 0; i < list.size(); i++)
    {
        output_vec.push_back(cv::Point2i(list[i].y, list[i].x));
    }

    return output_vec;
}

void print_shortest_path(std::vector<Node*> path_vec)
{
    for(int i = path_vec.size() -1; i >= 0; i--)
    {
        std::cout << " X: " << path_vec[i]->x_coordinate << "Y: " << path_vec[i]->y_coordinate << std::endl;
    }

}


int main()
{
    cv::Mat floor_plan = cv::imread("floor_plan.png");
    std::vector<cv::Point2i> way_points;
    getWaypoints(floor_plan, way_points);
    way_points = switch_XY_coordinates(way_points);

    for(unsigned int i = 0; i < way_points.size(); i++)
    {
        std::cout << "X: " << way_points[i].x << "Y: " << way_points[i].y <<std::endl;
         cv::circle(floor_plan, way_points[i], 2, cv::Scalar(0,0,255), -1);
    }

    Graph graph_of_map(floor_plan, way_points);
    vector<Node*> shortest_path = graph_of_map.djikstra_search(8, 7, 112, 71);
    print_shortest_path(shortest_path);

    for(unsigned int i = 0; i < shortest_path.size() - 1; i++)
    {
        cv::Point2i point1 = cv::Point2i(shortest_path[i]->x_coordinate, shortest_path[i]->y_coordinate);
        cv::Point2i point2 = cv::Point2i(shortest_path[i+1]->x_coordinate, shortest_path[i+1]->y_coordinate);
        cv::line(floor_plan, point1, point2, cv::Scalar(255,0,0));
    }

    cv::namedWindow("test", cv::WINDOW_NORMAL);
    cv::imshow("test", floor_plan);
    cv::waitKey(0);

    return 0;
}

