#ifndef CAMERA_H
#define CAMERA_H
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"
#include <boost/math/constants/constants.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>


void cameraCallback(ConstImageStampedPtr &msg);


#endif // CAMERA_H
