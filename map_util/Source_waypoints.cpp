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

	cv::Mat moar_waypoints = img_floor_plan.clone();
	cv::Mat gray;
	cv::cvtColor(img_floor_plan, gray, cv::COLOR_BGR2GRAY);
	
	std::vector<cv::Point2i> corners;


	cv::Mat img_floor_waypoints = img_floor_plan.clone();
	std::vector<std::vector<int>> gradient_vector(img_floor_plan.cols, std::vector<int>(img_floor_plan.rows, 0));

	//cv::Mat img_only_waypoints(img_floor_plan.rows, img_floor_plan.cols, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat img_only_waypoints(img_floor_plan.rows, img_floor_plan.cols, CV_8UC1, cv::Scalar(255));





	std::cout << "Testing gradient-map" << std::endl;
	cv::Mat hello;
	getGradientMap(gray, img_floor_plan, gradient_vector);
	std::cout << "No errors" << std::endl;
	std::vector<cv::Point2i> waypoints;
	//getWaypoints(img_floor_waypoints, gradient_vector, waypoints);

	getWaypoints(img_only_waypoints, gradient_vector, waypoints);

	//getCorners(img_only_waypoints, corners);

	cv::Mat img_only_reduced_waypoints(img_floor_plan.rows, img_floor_plan.cols, CV_8UC1, cv::Scalar(255));

	for (auto& point : corners)
	{
		//std::cout << point << std::endl;
		reduced_waypoints.at<cv::Vec3b>(point.x, point.y) = cv::Vec3b(0, 0, 255);
		img_only_reduced_waypoints.at<uchar>(point.x, point.y) = 0;
	}

	std::vector<cv::Point2i> grouped_waypoints;

	//groupWaypoints(img_only_reduced_waypoints, 6, 3, grouped_waypoints);
	groupWaypoints(img_only_waypoints, 20, 5, grouped_waypoints);

	std::cout << "Size of grouped waypoints " << grouped_waypoints.size() << std::endl;

	for (auto& point : grouped_waypoints)
	{
		//std::cout << point << std::endl;
		moar_waypoints.at<cv::Vec3b>(point.x, point.y) = cv::Vec3b(0, 127, 255);
		//img_only_reduced_waypoints.at<uchar>(point.x, point.y) = 0;
	}

	// grouped waypoints contains 

	showImage("Less waypoints", moar_waypoints);
	showImage(floor_plan_with_corners, img_floor_plan);
	showImage("Waypoints", img_only_waypoints);
	showImage("Reduced Waypoints", img_only_reduced_waypoints);
	showImage("Floor plan", reduced_waypoints);

	

	cv::waitKey(0);

	return 0;
}



