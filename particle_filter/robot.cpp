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

extern double gaz_x_pos;
extern double gaz_y_pos;

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

//      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
//                << _msg->pose(i).position().x() << std::setw(6)
//                << _msg->pose(i).position().y() << std::endl; //std::setw(6)
//                << _msg->pose(i).position().z() << std::setw(6)
//                << _msg->pose(i).orientation().w() << std::setw(6)
//                << _msg->pose(i).orientation().x() << std::setw(6)
//                << _msg->pose(i).orientation().y() << std::setw(6)
//                << _msg->pose(i).orientation().z() << std::endl;
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

      double goal_vec_x = goal_x - x_pos;
      double goal_vec_y = goal_y - y_pos;

      gaz_x_pos = x_pos;
      gaz_y_pos = y_pos;

      double ori_x = cos(robot_yaw);
      double ori_y = sin(robot_yaw);




      global_goal_angle = (atan2(goal_vec_y,goal_vec_x) - atan2(ori_y, ori_x));

      if (global_goal_angle > pi)
      {
          global_goal_angle -= 2*pi;
      }
      else if (global_goal_angle < -pi)
      {
          global_goal_angle += 2*pi;
      }

      //std::cout << "Angle towards goal: " << global_goal_angle << std::endl;
      //std::cout << "Robot orientation: " << robot_yaw << std::endl;
    }
  }
}
