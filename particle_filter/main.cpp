#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"
#include <boost/math/constants/constants.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>

/* Driver includes */
#include "camera.h"
#include "lidar.h"
#include "robot.h"
#include "gazebo.h"
#include "fuzzy_controller.h"
#include "../map_util/map_util.h"
#include "../localization/localize.h"
//#include "../emulated_lidar_scanner/emulated_lidar_scanner.cpp"
#include "emulated_lidar_scanner.h"

static boost::mutex da_mutex;

int tick = 0;
double goal_x;
double goal_y;
double global_minDist;
double global_angle;
double global_goal_angle;

std::default_random_engine da_generator;
std::vector<double> lidar_data(200, 0.0);
extern double first_data_point;

int main(int _argc, char **_argv) {

  //init_gazebo();

    std::vector<double> lidar_data;

    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr statSubscriber =
        node->Subscribe("~/world_stats", statCallback);

    gazebo::transport::SubscriberPtr poseSubscriber =
        node->Subscribe("~/pose/info", poseCallback);

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
    goal_x = 35.0;
    goal_y = 2.0;

    // Array that has direction and speed from control functions
    float arrSteer[2];

    const int key_left = 81;

    //Generate initial particles
    cv::Mat map = cv::imread("floor_plan.png");
    cv::resize(map, map, cv::Size(), 6, 6, cv::INTER_AREA);

    std::vector<Particle> particles;
    initParticles(map, 50, particles);

    int iteration = 0;
    //init_video_capture();
    // Loop
    while (true)
    {
        gazebo::common::Time::MSleep(10);

        da_mutex.lock();
        int key = cv::waitKey(1);
        da_mutex.unlock();

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

            //speed = 0.0;
            //dir = 0.0;

            predictParticles(map, particles, speed, dir); //speed*3

            //std::cout << "Size of lidar data (in main): " << lidar_data.size() << std::endl;
            //std::cout << "Global angle: " << global_goal_angle << std::endl;
            //std::cout << "First data point: " << first_data_point << std::endl;
            //std::cout << "First data point from main: " << lidar_data[0] << std::endl;

            for (auto & p : particles)
            {
                p.weight = error_lidar(emulate_lidar_ouput(map, p.col, p.row, p.orientation));

            }
            normaliseWeights(particles);

            resampleParticles(particles);

            //std::cout << "Particles size: " << particles.size() << std::endl;
            //std::cout << "Iteration number" << iteration++ << std::endl;
            //std::cout << "Particle[0]: " << particles[0].col << ", row: " << particles[0].row << std::endl;

            cv::Mat map_particles = map.clone();
            for (auto & p : particles)
            {
                cv::circle(map_particles, cv::Point2i((int)p.col,(int)p.row), 2, cv::Scalar(0,127,0), 2, 8);
            }

            showImage("Particles", map_particles);

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
