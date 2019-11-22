#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"
#include <boost/math/constants/constants.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <random>
#include <fstream>

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
double gaz_x_pos;
double gaz_y_pos;
double map_pixel_width;
double map_pixel_height;
double map_scale;

std::default_random_engine da_generator;
std::vector<double> lidar_data(200, 0.0);
extern double first_data_point;


int main(int _argc, char **_argv) {

    //init_gazebo()

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

    std::ofstream pose_file;
    pose_file.open("poses.csv");
    //pose_file << "my first write\n";
    //pose_file.close();


    // Array that has direction and speed from control functions
    float arrSteer[2];

    const int key_left = 81;

    //Load in the map
    cv::Mat map = cv::imread("floor_plan.png");
    // Generate waypoints
    std::vector<cv::Point2i> waypoints;
    getWaypoints(map, waypoints);

    //double t_x = (targetWaypoint.y - 120 / 2)/1.417;
    //double t_y = -(targetWaypoint.x - 80 / 2)/1.417;

    goal_x = (waypoints[32].y - map.cols / 2)/1.417;
    goal_y = -(waypoints[32].x - map.rows / 2)/1.417;

    //goal_x = 35;
    //goal_y = -20;

    // Resize map
    map_scale = 6.0;
    cv::resize(map, map, cv::Size(), 6, 6, cv::INTER_AREA);
    cv::Mat map_poses = map.clone();

    // Save the current size of the map, used by some util
    map_pixel_height = map.rows;
    map_pixel_width = map.cols;


    int save_pose_every = 5;
    int pose_it = 0;

    std::vector<cv::Point2i> real_pose;
    std::vector<cv::Point2i> estimated_pose;

    //Generate initial particles
    std::vector<Particle> particles;
    initParticles(map, 50, particles);

    int iteration = 0;

    int index_waypoint = 0;

    //std::vector<cv::Point2i> seq_waypoints = {waypoints[32], waypoints[38], waypoints[37], waypoints[33]};
    std::vector<cv::Point2i> seq_waypoints = {waypoints[32], waypoints[35], waypoints[34], waypoints[29], waypoints[27], waypoints[25], waypoints[22]};

    //goal_x = seq_waypoints[0].y - map.cols / 2;
    //goal_y = seq_waypoints[0].x - map.rows / 2;

    //init_video_capture();
    // Loop
    while (true)
    {
        gazebo::common::Time::MSleep(10);



        da_mutex.lock();
        int key = cv::waitKey(1);
        da_mutex.unlock();

        /*if(key == key_left)
        {
            break;
        }*/
        if(tick > 0)
        {
            simple_fuzzy_avoidance(arrSteer);

            pose_it++;

            // Get speed and direction from the control function
            speed = arrSteer[0];
            dir = arrSteer[1];

            std::cout << "goal angle: " << global_goal_angle << std::endl;
            //std::cout << "filler" << std::endl;

//            if (global_goal_angle > 0.4)
//                dir = -0.4;
//            else if (global_goal_angle < -0.4)
//                dir = 0.4;
//            else
//                dir = arrSteer[1];


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

            if (nearTargetWaypoint(seq_waypoints[index_waypoint], 2) && index_waypoint < seq_waypoints.size())
            {
                index_waypoint++;
                goal_x = (seq_waypoints[index_waypoint].y - 120 / 2)/1.417;
                goal_y = -(seq_waypoints[index_waypoint].x - 80 / 2)/1.417;

            }
            ;
            // " -- Current: x" << waypoints[32].x << " -- y: " << waypoints[32].y <<
            //std::cout << "Goal: x: " << goal_x << " -- y: " << goal_y << " -- near waypoint: " << nearTargetWaypoint(waypoints[32], 5) << std::endl;

            showImage("Particles", map_particles);

            if(pose_it > save_pose_every)
            {
                pose_it = 0;
                double xx = (gaz_x_pos*6*1.417 + map_pixel_width/2);
                double yy = (-gaz_y_pos*6*1.417 + map_pixel_height/2);

                double xxx = 0;
                double yyy = 0;

                //void getParticlesEstimatedPosition(std::vector<Particle>& particles, double &x, double &y)
                getParticlesEstimatedPosition(particles, xxx, yyy);

                pose_file << xx << ";" << yy << ";" << xxx << ";" << yyy << ";\n";
                //pose_file << "2;\n";
                cv::circle(map_poses, cv::Point2i((int)xxx,(int)yyy), 2, cv::Scalar(255,0,0), 2, 8);
                cv::circle(map_poses, cv::Point2i((int)xx,(int)yy), 4, cv::Scalar(255,0,255), 2, 8);

                //std::vector<cv::Point2i> real_pose;
            }
            if (index_waypoint == seq_waypoints.size() - 2)
            {
                pose_file.close();
                return 0;
            }
            showImage("Poses", map_poses);

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
