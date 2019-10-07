#ifndef CAMERA_H
#define CAMERA_H
<<<<<<< HEAD
=======
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"
#include <boost/math/constants/constants.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

void init_video_capture();

void destroy_video_capture();
>>>>>>> refs/remotes/origin/master

void cameraCallback(ConstImageStampedPtr &msg);

#endif // CAMERA_H
