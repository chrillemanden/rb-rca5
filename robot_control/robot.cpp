#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"
#include <boost/math/constants/constants.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

/*Driver includes */
#include "robot.h"

extern double goal_x;
extern double goal_y;

extern double global_goal_angle;

const double pi = boost::math::constants::pi<double>();

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  // std::cout << "Pose size: " << _msg->pose_size() << std::endl;
  for (int i = 0; i < _msg->pose_size(); i++) {
     //std::cout << "name: " << _msg->pose(i).name() << std::endl;
    if (_msg->pose(i).name() == "pioneer2dx") {

      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;
      //<< std::setw(6)//

      double p_ori_w = _msg->pose(i).orientation().w();
      double p_ori_x = _msg->pose(i).orientation().x();
      double p_ori_y = _msg->pose(i).orientation().y();
      double p_ori_z = _msg->pose(i).orientation().z();

      double siny_cosp = 2.0 * (p_ori_w * p_ori_z + p_ori_x * p_ori_y);
      double cosy_cosp = 1.0 - 2.0 * (p_ori_y * p_ori_y + p_ori_z * p_ori_z);
      double robot_yaw = atan2(siny_cosp, cosy_cosp);

      double x_pos = _msg->pose(i).position().x();
      double y_pos = _msg->pose(i).position().y();

      double x_delta = x_pos - goal_x;
      double y_delta = y_pos - goal_y;

      double angle2 = atan2(y_delta, -x_delta);
//      if (angle2 > pi)
//      {
//          angle
//      }
      global_goal_angle = (angle2 - robot_yaw)*-1;

      std::cout << "Angle atan2 " << angle2 << std::endl;
      std::cout << "Angle towards goal: " << global_goal_angle << std::endl;
      std::cout << "Robot orientation: " << robot_yaw << std::endl;
    }
  }
}
