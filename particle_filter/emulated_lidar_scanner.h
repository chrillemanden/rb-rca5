#ifndef EMULATED_LIDAR_SCANNER_H
#define EMULATED_LIDAR_SCANNER_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

std::vector<double> emulate_lidar_ouput(cv::Mat& map, int x_pos, int y_pos, double orientation);
double error_lidar(std::vector<double> measured_data, std::vector<double> calculated_data);
void plot_lidar_input(std::vector<double> lidar_input);


#endif // EMULATED_LIDAR_SCANNER_H
