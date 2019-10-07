#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"
#include <boost/math/constants/constants.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

/* Driver includes */
#include "lidar.h"

const double pi = boost::math::constants::pi<double>();

extern double global_minDist;
extern double global_angle;
extern int tick;
static boost::mutex mutex; //Maybe add extern

void simpleLidarCallback(ConstLaserScanStampedPtr &msg)
{

    //std::cout << "Hello!" << std::endl;
    float angle_min = float(msg->scan().angle_min());
    //  double angle_max = msg->scan().angle_max();
    float angle_increment = float(msg->scan().angle_step());

    float range_min = float(msg->scan().range_min());
    float range_max = float(msg->scan().range_max());

    int nranges = msg->scan().ranges_size();

    float range = *std::min_element(msg->scan().ranges().begin(), msg->scan().ranges().end());
    //std::cout << "Lowest range is: " << range << std::endl;

    //
    std::vector<double> half_circle_range;
    //std::cout << "pi: " << pi << std::endl;
    //std::cout << "nranges: " << nranges << std::endl;

    for (int i = 0; i < nranges; i++) {
        //std::cout << "half pi: " << -0.5 * pi << std::endl;
        //std::cout << "current val: " << msg->scan().ranges(i) << std::endl;
        if (angle_min + i * angle_increment >= -0.5 * pi && angle_min + i * angle_increment <= 0.5 * pi)
        {
            half_circle_range.push_back(msg->scan().ranges(i));
        }
    }
    //std::cout << "Size of new range: " << half_circle_range.size() << std::endl;
    double min_dist = *std::min_element(half_circle_range.begin(), half_circle_range.end());
    //std::cout << "Min dist in half-circle is: " << min_dist << std::endl;
    int min_dist_index = std::min_element(half_circle_range.begin(),half_circle_range.end()) - half_circle_range.begin();
    //std::cout << "Index: " << min_dist_index << std::endl;
    double min_dist_angle = (-0.5 * pi + angle_increment * min_dist_index) * -1;
    std::cout << "Angle of min dist in half-circle is: " << min_dist_angle << std::endl;

    global_minDist = min_dist;
    if (global_minDist > 10)
        global_minDist = 10;
    global_angle = min_dist_angle;
    tick++;
}


void lidarCallback(ConstLaserScanStampedPtr &msg) {

  //  std::cout << ">> " << msg->DebugString() << std::endl;
  float angle_min = float(msg->scan().angle_min());
  //  double angle_max = msg->scan().angle_max();
  float angle_increment = float(msg->scan().angle_step());

  float range_min = float(msg->scan().range_min());
  float range_max = float(msg->scan().range_max());

  int sec = msg->time().sec();
  int nsec = msg->time().nsec();

  int nranges = msg->scan().ranges_size();
  int nintensities = msg->scan().intensities_size();

  assert(nranges == nintensities);

  int width = 400;
  int height = 400;
  float px_per_m = 200 / range_max;

  cv::Mat im(height, width, CV_8UC3);
  im.setTo(0);
  for (int i = 0; i < nranges; i++) {
    float angle = angle_min + i * angle_increment;
    float range = std::min(float(msg->scan().ranges(i)), range_max);
    //    double intensity = msg->scan().intensities(i);
    cv::Point2f startpt(200.5f + range_min * px_per_m * std::cos(angle),
                        200.5f - range_min * px_per_m * std::sin(angle));
    cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                      200.5f - range * px_per_m * std::sin(angle));
    cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
             cv::LINE_AA, 4);

    //    std::cout << angle << " " << range << " " << intensity << std::endl;
  }
  cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
  cv::putText(im, std::to_string(sec) + ":" + std::to_string(nsec),
              cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.0,
              cv::Scalar(255, 0, 0));

  mutex.lock();
  cv::imshow("lidar", im);
  mutex.unlock();
}
