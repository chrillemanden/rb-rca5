#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/>
#include <opencv2/features2d.hpp>

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <random>
#include <cstdlib>
#include <stdint.h>

//Exercise includes 
//#include "exercise3.h"
//#include "exercise5.h"

// AI project includes
#include "map_util.h"

//#define WHITE 255
//#define BLACK 0
//#define IMG_END 
//#define 



int main(int argc, char** argv) {

	(void)argc;
	(void)argv;

	std::string floor_plan_string = "Original floor plan";
	std::string floor_plan_with_corners = "Floor plan with corner detection";

	cv::Mat img_floor_plan = cv::imread("floor_plan.png");//, cv::IMREAD_GRAYSCALE);

	cv::Mat reduced_waypoints = img_floor_plan.clone();




	/* Get corners of floor plan */
	cv::Mat gray_map;
	cv::cvtColor(img_floor_plan, gray_map, cv::COLOR_BGR2GRAY);
	


	// Grid size
	int grid_width = 10;
	int grid_height = 6;

	cv::Mat only_grid;
	cv::Mat img_floor_plan_grid;

	// Create the grid for the map with the chosen grid size
	createGridMap(img_floor_plan, grid_width, grid_height, only_grid, img_floor_plan_grid);

	showImage("Only grid", only_grid);
	showImage("Gridified map", img_floor_plan_grid);

	std::vector<cv::Point2i> visual_actions;

	std::vector<std::vector<std::vector<bool>>> grid_actions(grid_height-1, std::vector<std::vector<bool>>(grid_width-1, std::vector<bool>(4, false)));

	// Get the valid grid actions for the current grid map
	getValidGridActions(only_grid, grid_width, grid_height, gray_map, visual_actions, grid_actions);


	// Add the visualised actions to the map
	for (auto& point : visual_actions)
	{
		//std::cout << point << std::endl;
		img_floor_plan_grid.at<cv::Vec3b>(point.x, point.y) = cv::Vec3b(255, 255, 0);
	}

	showImage("Gridified map with possible actions", img_floor_plan_grid);

	// Print out available actions 
	for (int col = 0; col < 9; col++)
	{
		for (int row = 0; row < 5; row++)
		{
			std::cout << "(x, y): (" << row << ", " << col << "), grid_action: [" << grid_actions[row][col][0] << ", " << grid_actions[row][col][1] << ", " << grid_actions[row][col][2] << ", " << grid_actions[row][col][3] << "]" << std::endl;
		}
	}

	cv::waitKey(0);

	return 0;
}



