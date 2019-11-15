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

#include "map_util.h"


void showImage(std::string image_name, cv::Mat mat)
{
	cv::namedWindow(image_name, cv::WINDOW_NORMAL);
	imshow(image_name, mat);
}

void getCorners(cv::Mat& mat, std::vector<cv::Point2i>& corners)
{
	for (int col = 0; col < mat.cols; col++)
	{
		for (int row = 0; row < mat.rows; row++)
		{
			int curr_pixel = (int)mat.at<uchar>(row, col);

			//std::cout << (int)mat.at<uchar>(row, col) << " ";

			/* If white then not a corner */
			if (curr_pixel)
				continue;

			int rows = mat.rows - 1;
			int cols = mat.cols - 1;

			/* Special border corner cases */
			if ((col == 0 && row == 0) || (col == 0 && row == rows) || (col == cols && row == 0) || (col == cols && row == rows))
			{
				// These corners don't matter for the algorithm
				continue;
			}

			/* Special border cases */
			// Left side
			if (col == 0)
			{
				if (!(int)mat.at<uchar>(row, col + 1) )
				{
					corners.push_back(cv::Point2i(row, col));
					continue;
				}
				continue;
			}
			// Top side 
			else if (row == 0)
			{
				if (!(int)mat.at<uchar>(row + 1, col))
				{
					corners.push_back(cv::Point2i(row, col));
					continue;
				}
				continue;
			}
			// Right side
			else if (col == cols)
			{
				if (!(int)mat.at<uchar>(row, col - 1))
				{
					corners.push_back(cv::Point2i(row, col));
					continue;
				}
				continue;
			}
			// Bottom side
			else if (row == rows)
			{
				if (!(int)mat.at<uchar>(row - 1, col))
				{
					corners.push_back(cv::Point2i(row, col));
					continue;
				}
				continue;
			}


			/* Rules for all other pixels */

			// Adjacent pixels
			int l_pixel = (int)mat.at<uchar>(row, col - 1);
			int r_pixel = (int)mat.at<uchar>(row, col + 1);
			int u_pixel = (int)mat.at<uchar>(row - 1, col);
			int d_pixel = (int)mat.at<uchar>(row + 1, col);
			// Diagonal pixels
			int ul_pixel = (int)mat.at<uchar>(row - 1, col - 1);
			int ur_pixel = (int)mat.at<uchar>(row - 1, col + 1);
			int dr_pixel = (int)mat.at<uchar>(row + 1, col + 1);
			int dl_pixel = (int)mat.at<uchar>(row + 1, col - 1);

			// Special rules for walls that are thicker than 1 pixel
			if ((!l_pixel && !r_pixel) && ((!ul_pixel && !ur_pixel) || (!dl_pixel && !dr_pixel)))
			{
				continue;
			}
			else if ((!u_pixel && !d_pixel) && ((!ul_pixel && !dl_pixel) || (!ur_pixel && !dr_pixel)))
			{
				continue;
			}

			// Finds the rest of corners
			if ((l_pixel ^ r_pixel) || (u_pixel ^ d_pixel))
			{
				corners.push_back(cv::Point2i(row, col));
				continue;
			}

			// Find crosses
			if (l_pixel & r_pixel & u_pixel & d_pixel)
			{
				corners.push_back(cv::Point2i(row, col));
				continue;
			}


		}

	}
}


void getGridIntersections(cv::Mat& mat, std::vector<cv::Point2i>& corners)
{
	for (int col = 0; col < mat.cols; col++)
	{
		for (int row = 0; row < mat.rows; row++)
		{
			int curr_pixel = (int)mat.at<uchar>(row, col);

			//std::cout << (int)mat.at<uchar>(row, col) << " ";

			/* If white then not a corner */
			if (curr_pixel)
				continue;

			int rows = mat.rows - 1;
			int cols = mat.cols - 1;

			//std::cout << "Special border corner cases " << std::endl;
			/* Special border corner cases */
			if ((col == 0 && row == 0) || (col == 0 && row == rows) || (col == cols && row == 0) || (col == cols && row == rows))
			{
				// These corners don't matter for the algorithm
				continue;
			}
			//std::cout << "Special border cases " << std::endl;
			//std::cout << "row: " << row << ", col:" << col << std::endl;
			/* Special border cases */
			// Left side
			if (col == 0)
			{
				if (!(int)mat.at<uchar>(row, col + 1))
				{
					if ((!(int)mat.at<uchar>(row - 1, col)) && (!(int)mat.at<uchar>(row + 1, col)))
					{
						corners.push_back(cv::Point2i(row, col));
						continue;
					}
				}
				continue;
			}
			// Top side 
			else if (row == 0)
			{
				if (!(int)mat.at<uchar>(row + 1, col))
				{
					if ((!(int)mat.at<uchar>(row, col + 1)) && (!(int)mat.at<uchar>(row, col - 1)))
					{
						corners.push_back(cv::Point2i(row, col));
						continue;
					}
				}
				continue;
			}
			// Right side
			else if (col == cols)
			{
				if (!(int)mat.at<uchar>(row, col - 1))
				{
					if ((!(int)mat.at<uchar>(row - 1, col)) && (!(int)mat.at<uchar>(row + 1, col)))
					{
						corners.push_back(cv::Point2i(row, col));
						continue;
					}
				}
				continue;
			}
			// Bottom side
			else if (row == rows)
			{
				if (!(int)mat.at<uchar>(row - 1, col))
				{
					if ((!(int)mat.at<uchar>(row, col - 1)) && (!(int)mat.at<uchar>(row, col + 1)))
					{
						corners.push_back(cv::Point2i(row, col));
						continue;
					}
				}
				continue;
			}


			/* Rules for all other pixels */

			// Adjacent pixels
			//std::cout << "Find the rest of corners " << std::endl;
			int l_pixel = (int)mat.at<uchar>(row, col - 1);
			int r_pixel = (int)mat.at<uchar>(row, col + 1);
			int u_pixel = (int)mat.at<uchar>(row - 1, col);
			int d_pixel = (int)mat.at<uchar>(row + 1, col);

			// Finds the rest of corners
			if ((l_pixel ^ r_pixel) || (u_pixel ^ d_pixel))
			{
				corners.push_back(cv::Point2i(row, col));
				continue;
			}

			// Find crosses
			if (!l_pixel & !r_pixel & !u_pixel & !d_pixel)
			{
				corners.push_back(cv::Point2i(row, col));
				continue;
			}


		}

	}
}

void getGridMidpoints(std::vector<cv::Point2i>& intersections, int grid_width, int grid_height, std::vector<cv::Point2i>& midpoints)
{

	for (int col = 0; col < grid_width - 1; col++)
	{
		for (int row = 0; row < grid_height - 1; row++)
		{
			int x = (intersections[row].x + intersections[row + 1].x) / 2;
			int y = (intersections[col * grid_height].y + intersections[(col + 1) * grid_height].y) / 2;
			midpoints.push_back(cv::Point2i(x, y));
		}
	}
}

// void getValidGridActions(std::vector<cv::Point2i>& midpoints, std::vector<cv::Point2i>& intersections, cv::Mat& map, std::vector<cv::Point2i>& directions, std::vector<std::vector<std::vector<bool>>>& action_grid)
void getValidGridActions(cv::Mat& grid_map, int grid_width, int grid_height, cv::Mat& map, std::vector<cv::Point2i>& directions, std::vector<std::vector<std::vector<bool>>>& action_grid)
// Given a gridified map find available actions
/*
	Input
	grid_map is a gray image where everything is white except the grid which is black
	grid_width: number of grid lines along the y-axis
	grid_height: number of grid lines along the x-axis
	map is the gray version of the original floor plan
	Output
	directions is vector containing the points that can be used to visualise the valid actions
	action_grid is a 3D vector that holds information about valid actions
		action_grid
		indexes for the four actions
		0 - up
		1 - down
		2 - right
		3 - left
*/
{
	// Get the intersection points
	std::vector<cv::Point2i> intersections;
	getGridIntersections(grid_map, intersections);

	// Get the midpoints in the grid
	std::vector<cv::Point2i> midpoints;
	getGridMidpoints(intersections, grid_width, grid_height, midpoints);

	int width = grid_width - 1;
	int height = grid_height - 1;


	for (int col = 0; col < width; col++)
	{
		for (int row = 0; row < height; row++)
		{
			cv::Point2i goal;
			int loc = row + col * height; // Index of current midpoint in the 1d vector
			int left_border = intersections[col*(height+1)].y;
			int right_border = intersections[(col + 1)*(height + 1)].y;
			int top_border = intersections[row].x;
			int bottom_border = intersections[row + 1].x;
			
			// Valid to go down? 
			if (row < height - 1)
			{
				goal = midpoints[loc + 1];
				
				bool triedLeft = false;
				bool triedRight = false;

				int x = midpoints[loc].x;
				int y = midpoints[loc].y;

				while (true)
				{
					// Check if there is obstacle
					
					
					if (map.at<uchar>(x, y) == 0)
					{
						x--; // Obstacle, so go up
						
						// Try to go left
						if (map.at<uchar>(x, y-1) == 255 && (y-1) > left_border && !triedLeft)
							y--; // Go left
						else
							triedLeft = true; // Don't try to go left again
						
						// Try to go right if left failed
						if (map.at<uchar>(x, y+1) == 255 && (y+1) < right_border && triedLeft && !triedRight)
							y++; // Go right
						else if (triedLeft)
							triedRight = true;

						if (triedLeft && triedRight) // Tried both ways. Terminate
							break;

					}

					x++; // Go down

					// Check if need to get back on track
					if (y != goal.y)
						if (!triedLeft)
							if (map.at<uchar>(x, y + 1) == 255 && (y + 1) < right_border) // Possible to go right again?
								y++; // Go right
						else
							if (map.at<uchar>(x, y - 1) == 255 && (y - 1) > left_border) // Possible to go left again?
								y--; // Go left
									

					// Check if goal is reached
					// When on same row as goal assume goal is reached!
					if (x == goal.x)
					{
						directions.push_back(cv::Point2i(midpoints[loc].x + 2, midpoints[loc].y));
						directions.push_back(cv::Point2i(x - 2, midpoints[loc].y));
						
						// From this cell it is possible to go down
						action_grid[row][col][1] = true;
						// From the cell below it is possible to go up
						action_grid[row + 1][col][0] = true;
						break;
					}				
					
				}

			}

			
			// Valid to go right? 
			if (col < width - 1)
			{
				goal = midpoints[loc + height];

				bool triedUp = false;
				bool triedDown = false;
				
				int x = midpoints[loc].x;
				int y = midpoints[loc].y;

				while (true)
				{
					if (map.at<uchar>(x, y) == 0)
					{
						y--; // Obstacle so go back left
						
						// Try to go down
						if (map.at<uchar>(x + 1, y) == 255 && (x + 1) < bottom_border && !triedDown)
							x++; // Go down
						else
							triedDown = true; // Don't try to go down again

						// Try to go up if going down failed
						if (map.at<uchar>(x - 1, y) == 255 && (x - 1) > top_border && triedDown && !triedUp)
							x--; // Go up
						else if (triedDown)
							triedUp = true; // Don't try to go up again
					
						if (triedUp && triedDown) // Tried both ways. Terminate
							break;
					}
					
					y++; // Go right

					// Check if need to get back on track
					if (x != goal.x)
						if (!triedUp)
							if (map.at<uchar>(x + 1, y) == 255 && (x + 1) < bottom_border) // Possible to go down again?
								x++; // Go down
						else
							if (map.at<uchar>(x - 1, y) == 255 && (x - 1) > top_border) // Possible to go up again?
								x--; // Go up
					
					// Check if goal is reached
					// When on same col as goal assume goal is reached!
					if (y == goal.y)
					{
						directions.push_back(cv::Point2i(midpoints[loc].x, midpoints[loc].y + 2));
						directions.push_back(cv::Point2i(midpoints[loc].x, y - 2));

						// From this cell it is possible to go right
						action_grid[row][col][2] = true;
						// From the cell below it is possible to go left
						action_grid[row][col + 1][3] = true;

						break;
					}

					
					
				}
			}
		}
	}
}



void getGradientMap(cv::Mat& input, cv::Mat& output, std::vector<std::vector<int>>& vec)
{
	// BGR
	std::vector<cv::Vec3b> color_gradients = {
											cv::Vec3b(127, 0, 255), // Cyan
											cv::Vec3b(255, 0, 255), // Pink
											cv::Vec3b(255, 0, 127), // Purple
											cv::Vec3b(255, 0,   0), // Blue
											cv::Vec3b(255, 128, 0), // Light blue
											cv::Vec3b(255, 255, 0), // Turkish
											cv::Vec3b(0, 255,   0), // Green
											cv::Vec3b(0, 255, 255), // Yellow
											cv::Vec3b(0, 128, 255), // Orange
											cv::Vec3b(0,   0, 255)  // Red

	};

	// Input is gray image
	// Output is RGB image
	for (int col = 1; col < input.cols - 1; col++)
	{
		for (int row = 1; row < input.rows - 1; row++)
		{
			int curr_pixel = (int)input.at<uchar>(row, col);


			if (curr_pixel) // Pixel is not obstacle
			{
				// Lots of things to happen in here
				//std::cout << "This is white" << std::endl;
				int clearance_count = 1;
				bool obstacle = false;

				//std::cout << "row: " << row << " & col: " << col << std::endl;

				while (true)
				{
					obstacle = false;

					// Down and to the right
					for (int x = col - (clearance_count - 1); x < 2 * clearance_count + col - (clearance_count - 1); x++)
					{
						//std::cout << "Inside this for " << x << std::endl;
						if (!(int)input.at<uchar>(row + clearance_count, x))
							obstacle = true;
						//std::cout << "Down and to the right" << std::endl;
					}
					//std::cout << "Never here" << std::endl;
					// Right and up
					for (int y = row + (clearance_count - 1); y > row + (clearance_count - 1) - 2 * clearance_count; y--)
					{ // y = 1 + (1 - 1)  y > 1 + (1 - 1) - 2 * 1; y--
						if (!(int)input.at<uchar>(y, col + clearance_count))
							obstacle = true;
						//std::cout << "Right and up" << std::endl;
					}
					// Up and to the left
					for (int x = col + (clearance_count - 1); x > col + (clearance_count - 1) - 2 * clearance_count; x--)
					{
						if (!(int)input.at<uchar>(row - clearance_count, x))
							obstacle = true;
						//std::cout << "Up and to the left" << std::endl;
					}
					// Left and down
					for (int y = row - (clearance_count - 1); y < 2 * clearance_count + row - (clearance_count - 1); y++)
					{ // y = 1 - (1 - 1) ; y < 2 * 1 - 1 - (1 - 1)
						if (!(int)input.at<uchar>(y, col - clearance_count))
							obstacle = true;
						//std::cout << "Left and down" << std::endl;
					}
					//std::cout << "Loop" << std::endl;



					if (!obstacle)
						clearance_count++;
					else
						break;

					if (col - (clearance_count + 1) < 0 || row - (clearance_count + 1) < 0)
						break;
					if (col + (clearance_count + 1) > input.cols || row + (clearance_count + 1) > input.rows)
						break;

				}

				vec[col][row] = clearance_count;

				if (clearance_count > 10)
					clearance_count = 10;

				// Set the colour of the pixel
				output.at<cv::Vec3b>(row, col) = color_gradients[clearance_count - 1];


			}
			else // Pixel is obstacle
			{
				// Just add the black pixel to the output image
				//std::cout << "This is black" << std::endl;
			}

		}
	}
}
//std::vector<std::vector<int>>& vec
void getWaypoints(cv::Mat& map, std::vector<cv::Point2i>& waypoints)
{
	// Fill the map edges
	fillMapEdges(map);

	cv::Mat gray;
	cv::cvtColor(map, gray, cv::COLOR_BGR2GRAY);
	cv::Mat white(map.rows, map.cols, CV_8UC1, cv::Scalar(255));
	cv::Mat gradient_map = map.clone();

	

	std::vector<std::vector<int>> gradients(map.cols, std::vector<int>(map.rows, 0));
	
	getGradientMap(gray, gradient_map, gradients);

	std::vector<cv::Point2i> temp_waypoints;
	cv::Mat map_waypoints(map.rows, map.cols, CV_8UC1, cv::Scalar(255));

	// Find the local maxima in the gradient map
	for (int col = 1; col < map.cols - 1; col++)
	{
		for (int row = 1; row < map.rows - 1; row++)
		{
			std::vector<int> comp;
			//int curr_pixel = vec[row][col];
			//int l_pixel = vec[row][col - 1];
			//int r_pixel = vec[row][col + 1];
			//int u_pixel = vec[row - 1][col];
			//int d_pixel = vec[row + 1][col];
			//comp.push_back(vec[row][col]); // curr_pixel
			//comp.push_back(vec[row][col - 1]); // left pixel
			//comp.push_back(vec[row][col + 1]); // right pixel
			//comp.push_back(vec[row - 1][col]); // up pixel
			//comp.push_back(vec[row + 1][col]); // down pixel
			//comp.push_back(vec[row - 1][col - 1]); // up left pixel
			//comp.push_back(vec[row - 1][col + 1]); // up right pixel
			//comp.push_back(vec[row + 1][col + 1]); // down right pixel
			//comp.push_back(vec[row + 1][col - 1]); // down left pixel

			comp.push_back(gradients[col][row]); // curr_pixel
			comp.push_back(gradients[col][row - 1]); // left pixel
			comp.push_back(gradients[col][row + 1]); // right pixel
			comp.push_back(gradients[col - 1][row]); // up pixel
			comp.push_back(gradients[col + 1][row]); // down pixel
			comp.push_back(gradients[col - 1][row - 1]); // up left pixel
			comp.push_back(gradients[col - 1][row + 1]); // up right pixel
			comp.push_back(gradients[col + 1][row + 1]); // down right pixel
			comp.push_back(gradients[col + 1][row - 1]); // down left pixel

			if (comp[0] == *std::max_element(comp.begin(), comp.end()) && comp[0] != 0 && comp[0] != 1)
			{
				// Colorize the output image
				//output.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 0);
				white.at<uchar>(row, col) = 0;
				// Add to list of coordinates
				//temp_waypoints.push_back(cv::Point2i(row, col));
			}
		}
	}

	// Group the local maxima into waypoints
	groupWaypoints(white, 20, 3, waypoints);

}

void fillMapEdges(cv::Mat& map)
{
	int col;

	for (int row = 0; row < map.rows; row++)
	{
		col = -1;
		// Check if next pixel is white
		while (map.at<cv::Vec3b>(row, col + 1) == cv::Vec3b(255, 255, 255))
		{
			std::cout << "this is white" << std::endl;
			std::cout << "row, col: (" << row << ", " << col + 1 << ")" << std::endl;
			map.at<cv::Vec3b>(row, col + 1) = cv::Vec3b(0, 0, 0);
			col++;
		}
	}

	

	
	for (int row = 0; row < map.rows; row++)
	{
		col = map.cols;
		// Check if next pixel is white
		while (map.at<cv::Vec3b>(row, col - 1) == cv::Vec3b(255, 255, 255))
		{
			map.at<cv::Vec3b>(row, col - 1) = cv::Vec3b(0, 0, 0);
			col--;
		}
	}

	

	int row;
	for (col = 0; col < map.cols; col++)
	{
		row = -1;
		// Check if next pixel is white
		while (map.at<cv::Vec3b>(row + 1, col) == cv::Vec3b(255, 255, 255))
		{
			map.at<cv::Vec3b>(row + 1, col) = cv::Vec3b(0, 0, 0);
			row++;
		}
	}

	//row = map.rows;
	for (col = 0; col < map.cols; col++)
	{
		row = map.rows;
		// Check if next pixel is white
		while (map.at<cv::Vec3b>(row - 1, col) == cv::Vec3b(255, 255, 255))
		{
			map.at<cv::Vec3b>(row - 1, col) = cv::Vec3b(0, 0, 0);
			row--;
		}
	}

	//showImage("Test", map);
}

bool pointFound(std::vector<int> disc_row, std::vector<int> disc_col, cv::Point2i point)
{
	for (int i = 0; i < disc_row.size(); i++)
	{
		if (disc_row[i] == point.x && disc_col[i] == point.y)
			return false;
	}
	return true;
}

void groupWaypoints(cv::Mat input, int max_iterations, int kernel_dim, std::vector<cv::Point2i>& new_waypoints)
{
	int n = 0;
	// Make a white copy of the original 
	cv::Mat white(input.rows, input.cols, CV_8UC1, cv::Scalar(255));
	cv::Mat copy = input.clone();

	while (n < max_iterations)
	{
		std::vector<cv::Point2i> grouped_waypoints;

		groupWaypointsOnce(copy, kernel_dim, grouped_waypoints);
		cv::Mat white(input.rows, input.cols, CV_8UC1, cv::Scalar(255));

		for (auto& point : grouped_waypoints)
		{
			white.at<uchar>(point.x, point.y) = 0;
			//std::cout << point << std::endl;
			//white.at<cv::Vec3b>(point.x, point.y) = cv::Vec3b(127, 127, 127);
			//img_only_reduced_waypoints.at<uchar>(point.x, point.y) = 0;
		}

		copy = white.clone();
		new_waypoints = grouped_waypoints;
		n++;
	}

	

}

void groupWaypointsOnce(cv::Mat input, int kernel_dim, std::vector<cv::Point2i> &new_waypoints)
{
	// Mat input is image with only waypoints
	
	int max_radius = kernel_dim;

	for (int col = 1; col < input.cols - 1; col++)
	{
		for (int row = 1; row < input.rows - 1; row++)
		{
			int curr_pixel = (int)input.at<uchar>(row, col);


			if (!curr_pixel) // Pixel is a point
			{
				// Lots of things to happen in here
				
				int clearance_count = 1;

				std::vector<int> disc_row;
				std::vector<int> disc_col;

				disc_row.push_back(row);
				disc_col.push_back(col);

				//std::cout << "(row, col) = (" << row << ", " << col << ")" << std::endl;

				while (clearance_count <= max_radius)
				{
					// Down and to the right
					for (int x = col - (clearance_count - 1); x < 2 * clearance_count + col - (clearance_count - 1); x++)
					{
						if (!(int)input.at<uchar>(row + clearance_count, x))
						{
							disc_row.push_back(row + clearance_count);
							disc_col.push_back(x);
							
						}
					}
					
					// Right and up
					for (int y = row + (clearance_count - 1); y > row + (clearance_count - 1) - 2 * clearance_count; y--)
					{ 
						if (!(int)input.at<uchar>(y, col + clearance_count))
						{
							disc_row.push_back(y);
							disc_col.push_back(col + clearance_count);						
						}
					}

					// Up and to the left
					for (int x = col + (clearance_count - 1); x > col + (clearance_count - 1) - 2 * clearance_count; x--)
					{
						if (!(int)input.at<uchar>(row - clearance_count, x))
						{
							disc_row.push_back(row - clearance_count);
							disc_col.push_back(x);
						}
					}

					// Left and down
					for (int y = row - (clearance_count - 1); y < 2 * clearance_count + row - (clearance_count - 1); y++)
					{ // y = 1 - (1 - 1) ; y < 2 * 1 - 1 - (1 - 1)
						if (!(int)input.at<uchar>(y, col - clearance_count))
						{
							disc_row.push_back(y);
							disc_col.push_back(col - clearance_count);
						}
					}

					
					clearance_count++;
					//std::cout << "clearance_count: " << clearance_count << std::endl;

					if (col - (clearance_count + 1) < 0 || row - (clearance_count + 1) < 0)
					{
						//std::cout << "Break out" << std::endl;
						
						break;
					}
					if (col + (clearance_count + 1) > input.cols || row + (clearance_count + 1) > input.rows)
					{
						//std::cout << "Break out" << std::endl;
						break;
					}

				}

				//std::cout << "Disc row size: " << disc_row.size() << std::endl;

				// Add the new grouped waypoint
				cv::Point2i new_mean = cv::Point2i((int)(accumulate(disc_row.begin(), disc_row.end(), 0) / disc_row.size()), (int)(accumulate(disc_col.begin(), disc_col.end(), 0) / disc_col.size()));
				new_waypoints.push_back(new_mean);
				
			}

		}
	}

}

//void findWaypointConnections(std::vector<cv::Point2i>& waypoints)
//{
//
//}


void createGridMap(cv::Mat& map, int grid_width, int grid_height, cv::Mat& grid, cv::Mat& map_grid_overlay)
{
	/* Make a gray version of the map */
	cv::Mat gray_map;
	cv::cvtColor(map, gray_map, cv::COLOR_BGR2GRAY);

	/* Create grid over map with corners */
	std::vector<cv::Point2i> corners;
	getCorners(gray_map, corners);

	/* Do k-means clustering on the corner points */
	std::vector<std::vector<int>> XY_list = split_XY(corners);
	std::vector<float> X_clusters = KMeansClustering(XY_list[0], grid_width, 10);
	std::vector<float> Y_clusters = KMeansClustering(XY_list[1], grid_height, 10);

	/* Draw the grid on a copy of the original map as well as on an white mat */
	cv::cvtColor(gray_map, map_grid_overlay, cv::COLOR_GRAY2BGR);
	cv::Mat white(map.rows, map.cols, CV_8UC1, cv::Scalar(255));
	grid = white.clone();

	//Draw horisontal lines
	for (int i = 0; i < Y_clusters.size(); i++)
	{
		cv::line(map_grid_overlay, cv::Point(0, cvRound(cvRound(Y_clusters[i]))), cv::Point(map_grid_overlay.cols, cvRound(cvRound(Y_clusters[i]))), cv::Scalar(0, 255, 0));
		cv::line(grid, cv::Point(0, cvRound(cvRound(Y_clusters[i]))), cv::Point(grid.cols, cvRound(cvRound(Y_clusters[i]))), cv::Scalar(0));
	}

	//Draw vertical lines
	for (int i = 0; i < X_clusters.size(); i++)
	{
		cv::line(map_grid_overlay, cv::Point(cvRound(X_clusters[i]), 0), cv::Point(cvRound(X_clusters[i]), map_grid_overlay.rows), cv::Scalar(0, 255, 0));
		cv::line(grid, cv::Point(cvRound(X_clusters[i]), 0), cv::Point(cvRound(X_clusters[i]), grid.rows), cv::Scalar(0));
	}
}


std::vector<float> init_means(int k, int max, int min)
{
	std::vector<float> means;
	for (
		int i = 0; i < k; i++)
	{
		means.push_back(min + i * (max - min) / (k - 1));
	}

	return means;
}

float get_distance(int dataPoint, float mean)
{
	float distance = abs((float)dataPoint - mean);

	return distance;
}

std::vector<float> update_mean(std::vector<float> means, std::vector<std::vector<int>> clustered_data)
{
	for (unsigned int i = 0; i < means.size(); i++)
	{
		float sum = std::accumulate(clustered_data[i].begin(), clustered_data[i].end(), 0);
		//std::cout << "sum: " << sum;
		if (clustered_data.size() > 0)
		{
			means[i] = sum / (float)clustered_data[i].size();
		}

	}


	return means;
}

std::vector<std::vector<int>> assign_to_mean(std::vector<float> means, std::vector<int> data)
{
	std::vector<int> empty_vec(0, 0);
	std::vector<std::vector<int>> output_vec(means.size(), empty_vec);

	for (unsigned int i = 0; i < data.size(); i++)
	{
		int index = 0;
		float distance;
		float min_distance = get_distance(data[i], means[0]);
		for (unsigned int j = 1; j < means.size(); j++)
		{
			distance = get_distance(data[i], means[j]);

			if (distance < min_distance) //add compare;
			{
				index = j;
				min_distance = distance;
			}
		}

		output_vec[index].push_back(data[i]);
	}

	return output_vec;
}

std::vector<float> KMeansClustering(std::vector<int> data, int k, int iterations)
{
	int min_list = *std::min_element(data.begin(), data.end());
	int max_list = *std::max_element(data.begin(), data.end());

	//Initialize clusters*/
	std::vector<float> means = init_means(k, max_list, min_list);



	//create histogram
	std::vector<int> hist_vec((max_list - min_list + 1), 0);

	for (unsigned int i = 0; i < data.size(); i++)
	{
		hist_vec[data[i] - min_list]++;
	}

	//print histogram

	//for (unsigned int i = 0; i < hist_vec.size(); i++)
	//{
	//	std::cout << hist_vec[i] << std::endl;
	//}

	//std::cout << "before iteration 0 " << std::endl;

	for (unsigned int j = 0; j < means.size(); j++)
	{
		//std::cout << means[j] << std::endl;
	}


	for (int i = 0; i < iterations; i++)
	{
		std::vector<std::vector<int>> clustered_data = assign_to_mean(means, data);
		means = update_mean(means, clustered_data);

		/*cout << "iteration: " << i << endl;
		for(unsigned int j = 0; j < means.size(); j++)
		{
			cout << means[j] << endl;
		}*/
	}

	//round means
	for (int i = 0; i < means.size(); i++)
	{
		means[i] = cvRound(means[i]);
	}

	//std::cout << "finished clustering" << std::endl;

	return means;
}

std::vector<std::vector<int>> split_XY(std::vector<cv::Point2i> point_list)
{

	std::vector<std::vector<int>> XY_list(2, std::vector<int>(0, 0));

	//std::cout << "point list size: " << point_list.size() << std::endl;
	for (unsigned int i = 0; i < point_list.size(); i++)
	{

		XY_list[0].push_back(point_list[i].y);
		XY_list[1].push_back(point_list[i].x); //X and Y apear to be switched


		//std::cout << "m- " << "X: " << point_list[i].x << " Y: " << point_list[i].y << std::endl;
	}

	//std::cout << "point list size: " << point_list.size() << std::endl;
	return XY_list;

}