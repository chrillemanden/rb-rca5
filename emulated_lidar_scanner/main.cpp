#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "emulated_lidar_scanner.cpp"

using namespace std;


/*std::vector<double> emulate_lidar_ouput(cv::Mat& map, int x_pos, int y_pos, double orientation)
{
    cv::Mat map_clone = map.clone();
    std::vector<double> lidar_ouput = {};

    double angle_inc = 2.268899/100.0;
    cv::Point2i robot_point = cv::Point2i(x_pos, y_pos);

    for(int i = -100; i <= 100; i++)
    {
        if(i == 0)
        {
            continue;
        }


        double cur_angle = orientation + angle_inc * (double) i;
        double max_laser_legth = 10.0 * 6.0; //6 is scalling factor.
        double cur_x = x_pos + max_laser_legth * cos(cur_angle);
        double cur_y = y_pos - max_laser_legth * sin(cur_angle);

        cv::Point2i laser_end_point = cv::Point2i((int) cur_x, (int) cur_y);

        //cv::circle(map, laser_end_point, 6, cv::Scalar(255,0,0), -1);
        std::cout << "start_point - " << robot_point << " endpoint - " << laser_end_point << std::endl;

        cv::LineIterator line_it(map, robot_point, laser_end_point, 8);

        std:cout << "length line - " << line_it.count << std::endl;

        for(int j = 0; j < line_it.count; line_it++, j++)
        {
            cv::Vec3b pixel_val = map.at<cv::Vec3b>(line_it.pos());
            //cout << "pixel val" << pixel_val << endl;
            //check if a wall pixel is encountered
            //if(pixel_val == cv::Vec3b(0,0,0));
            if(pixel_val.val[0] == 0 && pixel_val.val[1] == 0 && pixel_val.val[2] == 0)
            {
                cv::circle(map_clone, line_it.pos(), 2, cv::Scalar(0,0,255), -1);
                cout << "found black pixel at j - " << j << endl;
                double distance = sqrt(pow(line_it.pos().x-robot_point.x, 2.0)+pow(line_it.pos().y-robot_point.y, 2.0));
                lidar_ouput.push_back(distance/6.0); //missing unit
                break;
            }

            //store the connection if no wall pixels is encountered along the length of line
            if(j + 1 == line_it.count)
            {
                cv::circle(map_clone, line_it.pos(), 2, cv::Scalar(0,255,0), -1);
                double distance = sqrt(pow(line_it.pos().x-robot_point.x, 2.0)+pow(line_it.pos().y-robot_point.y, 2.0));
                lidar_ouput.push_back(distance/6.0); //missing unit
                break;
            }

         }


    }
    cv::imshow("Found points", map_clone);
    cv::waitKey(0);
    std::cout << "Length lidar length " << lidar_ouput.size() << std::endl;
    return lidar_ouput;

}

double error_lidar(std::vector<double> measured_data, std::vector<double> calculated_data)
{
    double sum_error = 0;
    for(unsigned int i = 0; i < measured_data.size(); i++)
    {
        double diff = measured_data[i] - calculated_data[i];
        double sq_error = pow(diff, 2.0);

        sum_error = sum_error + sq_error;
    }
    return sum_error;
}


void plot_lidar_input(std::vector<double> lidar_input)
{
    float angle_min = 2.268899;
    float angle_increment = (2.0 * 2.268899)/200.0;
    float range_min = 0.08;
    double range_max = 10.0;


    int nrange = lidar_input.size();

    int width = 400;
    int height = 400;
    float px_per_m = 200/range_max;

    cv::Mat im(height, width, CV_8UC3);
    im.setTo(0);

    for(int i = 0; i < nrange; i++)
    {
        float angle = angle_min + i * angle_increment;
        float range = std::min(lidar_input[i], range_max);
        cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                            200.5f - range_min * px_per_m * std::sin(angle));
        cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                          200.5f - range * px_per_m * std::sin(angle));
        cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
                 cv::LINE_AA, 4);
    }

     cv::imshow("lidar", im);
     cv::waitKey(0);
}
*/
int main()
{
    int scaling_factor = 6;
    cv::Mat map = cv::imread("floor_plan.png", cv::IMREAD_COLOR);
    cv::resize(map, map, cv::Size(0,0), scaling_factor, scaling_factor, cv::INTER_AREA);


    int x_cor = map.cols/2+150;
    int y_cor = map.rows/2;
    cv::circle(map, cv::Point2i(x_cor, y_cor), 2, cv::Scalar(255,0,0), -1);

    std::vector<double> lidar_data = emulate_lidar_ouput(map, x_cor, y_cor, 0);
    cv::imshow("map", map);
    for(int i = 0; i < lidar_data.size(); i++)
    {
        cout << lidar_data[i] << std::endl;
    }

    plot_lidar_input(lidar_data);

    //cv::imshow("lidar", map);
    //cv::waitKey(0);
    cout << "Hello World!" << endl;
    return 0;
}
