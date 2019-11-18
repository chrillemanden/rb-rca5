#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

/* 
	void getCorners()
	Given a map in the cv::Mat format, where black constitues obstacles and white constitues movable space find all corners of obstacles
	All the points of the corners are saved in a provided vector
*/
void showImage(std::string image_name, cv::Mat mat);

void getCorners(cv::Mat& mat, std::vector<cv::Point2i>& corners);


void getGridIntersections(cv::Mat& mat, std::vector<cv::Point2i>& corners);

void getGridMidpoints(std::vector<cv::Point2i>& intersections, int grid_width, int grid_height, std::vector<cv::Point2i>& midpoints);

void getValidGridActions(cv::Mat& grid_map, int grid_width, int grid_height, cv::Mat& map, std::vector<cv::Point2i>& directions, std::vector<std::vector<std::vector<bool>>>& action_grid);

void createGridMap(cv::Mat& map, int grid_width, int grid_height, cv::Mat& grid, cv::Mat& map_grid_overlay);

/*
	void getGradientMap()
	Calculates gradients for a map

	cv::Mat& input:					A map where black constitues obstacles and white constitues movable space find all corners of obstacles
	cv::Mat& output:				A map same dimensions as input where the gradients will be drawn
	std::vector<std::vector<int>>&	A vec where gradient values are stored
*/
void getGradientMap(cv::Mat& input, cv::Mat& output, std::vector<std::vector<int>>& vec);


/*
	void getWaypoints()
	Given a gradient map, reduces the gradient map to specific waypoints

	cv::Mat& output:					A map where the waypoints are drawn
	std::vector<std::vector<int>>& vec: An input vec with gradient values for the map
	std::vector<cv::Point2i> points:	A vector where the waypoints are saved
*/
void getWaypoints(cv::Mat& map, std::vector<cv::Point2i>& waypoints);

void fillMapEdges(cv::Mat& map);

bool pointFound(std::vector<int> disc_row, std::vector<int> disc_col, cv::Point2i point);


/*
	void groupWaypoints()
	Given a set of waypoints groups close waypoints into one waypoint 

*/

void groupWaypoints(cv::Mat input, int max_iterations, int kernel_dim, std::vector<cv::Point2i>& new_waypoints);
void groupWaypointsOnce(cv::Mat input, int kernel_dim, std::vector<cv::Point2i> & new_waypoints);



std::vector<float> init_means(int k, int max, int min);
float get_distance(int dataPoint, float mean);
std::vector<float> update_mean(std::vector<float> means, std::vector<std::vector<int>> clustered_data);
std::vector<std::vector<int>> assign_to_mean(std::vector<float> means, std::vector<int> data);
std::vector<float> KMeansClustering(std::vector<int> data, int k, int iterations);
std::vector<std::vector<int>> split_XY(std::vector<cv::Point2i> point_list);
