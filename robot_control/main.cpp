#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"
#include <boost/math/constants/constants.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

/* Driver includes */
#include "camera.h"
#include "lidar.h"
#include "robot.h"
#include "gazebo.h"
#include "fuzzy_controller.h"

static boost::mutex mutex;

int tick = 0;
double global_minDist;
double global_angle;


int main(int _argc, char **_argv) {

  //init_gazebo();


    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber =
        node->Subscribe("~/world_stats", statCallback);

    //gazebo::transport::SubscriberPtr poseSubscriber =
    //    node->Subscribe("~/pose/info", poseCallback);

    gazebo::transport::SubscriberPtr cameraSubscriber =
        node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

    gazebo::transport::SubscriberPtr lidarSubscriber =
        node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

    gazebo::transport::SubscriberPtr simpleLidarSubscriber =
        node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", simpleLidarCallback);

    // Publish to the robot vel_cmd topic
    gazebo::transport::PublisherPtr movementPublisher =
        node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

    // Publish a reset of the world
    gazebo::transport::PublisherPtr worldPublisher =
        node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);


    // Initialise fuzzy controller
    init_fuzzy_controller();

    float speed = 0.0;
    float dir = 0.0;

    // Array that has direction and speed from control functions
    float arrSteer[2];

    const int key_left = 81;


    //init_video_capture();
    // Loop
    while (true)
    {
        gazebo::common::Time::MSleep(10);

        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

        if(key == key_left)
        {
            break;
        }
        if(tick > 0)
        {
            simple_fuzzy_avoidance(arrSteer);

            // Get speed and direction from the control function
            speed = arrSteer[0];
            dir = arrSteer[1];

            tick--;
        }

    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
    }
  //destroy_video_capture();
  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
