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
double robot_orientation;

std::default_random_engine da_generator;
std::vector<double> lidar_data(200, 0.0);
extern double first_data_point;

void draw_waypoints(cv::Mat &map, std::vector<cv::Point2i> waypoints_to_draw)
{
    for (auto & point : waypoints_to_draw)
    {
        std::cout << "Point values: " << point.y << " - " << point.x << std::endl;
        cv::circle(map, cv::Point2i((point.y*6), (point.x*6)), 5, cv::Scalar(0,128,0), 9, 8);
    }
}


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

    //if (key)

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


    int save_pose_every = 20;
    int pose_it = 0;

    std::vector<cv::Point2i> real_pose;
    std::vector<cv::Point2i> estimated_pose;

    //Generate initial particles
    std::vector<Particle> particles;
    initParticles(map, 50, particles);

    int iteration = 0;

    int index_waypoint = 0;

    //std::vector<cv::Point2i> seq_waypoints = {waypoints[32], waypoints[38], waypoints[37], waypoints[33], waypoints[30], waypoints[28], waypoints[19]};

    //std::vector<cv::Point2i> seq_waypoints = {waypoints[19], waypoints[13], waypoints[12], waypoints[13], waypoints[10], waypoints[7], waypoints[5], waypoints[0], waypoints[5], waypoints[7], waypoints[3], waypoints[10], waypoints[14], waypoints[20], waypoints[26]};

    // Room 26_23
    std::vector<cv::Point2i> seq_waypoints = {waypoints[13], waypoints[11], waypoints[10], waypoints[14], waypoints[17], waypoints[20], waypoints[26], waypoints[20], waypoints[17], waypoints[18], waypoints[21], waypoints[23], waypoints[37]};

    // Room 0
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[13], waypoints[11], waypoints[10], waypoints[7], waypoints[5], waypoints[0], waypoints[37]};

    // Room 2
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[19], waypoints[11], waypoints[6], waypoints[1], waypoints[2], waypoints[37]};

    // Room 12
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[19], waypoints[13], waypoints[12], waypoints[8]};

    // Room 22
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[32], waypoints[35], waypoints[34], waypoints[29], waypoints[27], waypoints[25], waypoints[22], waypoints[12]};

    // Room 24
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[32], waypoints[35], waypoints[36], waypoints[31], waypoints[24], waypoints[12]};

    // Room 32_30
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[28], waypoints[32], waypoints[38], waypoints[37], waypoints[33], waypoints[30], waypoints[32], waypoints[35]};


    // Trying to provoke some mistakes:
    // Room 0 failing
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[13], waypoints[11], waypoints[10], waypoints[3], waypoints[7], waypoints[5], waypoints[0], waypoints[37]};

    // Room 2 - 4 turning in a tight corner
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[19], waypoints[11], waypoints[6], waypoints[1], waypoints[2], waypoints[4], waypoints[2], waypoints[37]};

    // Room 16 - 14 - tight turns
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[19], waypoints[16], waypoints[14], waypoints[10], waypoints[11], waypoints[12]};

    // Room 16 - 14 - tight turns
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[11], waypoints[10], waypoints[7], waypoints[5], waypoints[0], waypoints[12]};

    // Room 0
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[13], waypoints[11], waypoints[10], waypoints[7], waypoints[5], waypoints[0], waypoints[5], waypoints[7], waypoints[3], waypoints[37]};

    // Room 22
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[32], waypoints[35], waypoints[34], waypoints[29], waypoints[27], waypoints[25], waypoints[22], waypoints[25], waypoints[27], waypoints[12]};

    // Room 30_38_32 fail!
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[28], waypoints[30], waypoints[33], waypoints[37], waypoints[38], waypoints[32], waypoints[28], waypoints[35]};

    // Room 30_38_32
    //std::vector<cv::Point2i> seq_waypoints = {waypoints[28], waypoints[32], waypoints[30], waypoints[33], waypoints[37], waypoints[38], waypoints[35], waypoints[36]};

    goal_x = (seq_waypoints[0].y - 120 / 2)/1.417;
    goal_y = -(seq_waypoints[0].x - 80 / 2)/1.417;

    std::cout << map_pixel_height << " - " << map_pixel_width << std::endl;

    // Starting point:
    cv::circle(map_poses, cv::Point2i(map_pixel_width/2,map_pixel_height/2), 4, cv::Scalar(128,0,0), 2, 8);

    //draw_waypoints(map_poses, std::vector<cv::Point2i>(seq_waypoints.begin(), seq_waypoints.end()-1));

//    for (auto & point : seq_waypoints)
//    {
//        std::cout << "Point values: " << point.y << " - " << point.x << std::endl;
//       cv::circle(map_poses, cv::Point2i((point.y*6), (point.x*6)), 3, cv::Scalar(0,128,0), 4, 8);
//    }
    // cv::Point2i((point.y*6 - 80 / 2), (point.x*6 - 120 / 2))
    //xxx = (xxx - map_pixel_width/2)/1.417/6;
    //yyy = -(yyy - map_pixel_height/2)/1.417/6;


    double estimated_rot;

    cv::Mat map_particles;

    //goal_x = seq_waypoints[0].y - map.cols / 2;
    //goal_y = seq_waypoints[0].x - map.rows / 2;

    //init_video_capture();
    // Loop

    int timestep = 0;
    while (true)
    {
        gazebo::common::Time::MSleep(10);



        da_mutex.lock();
        int key = cv::waitKey(1);
        da_mutex.unlock();

        if(key == key_left)
        {
            pose_file.close();
            cv::imwrite("tracking.png", map_poses);
            cv::imwrite("particles.png", map_particles);
            return 0;
            break;
        }
        if(tick > 0)
        {
            simple_fuzzy_avoidance(arrSteer);

            pose_it++;

            // Get speed and direction from the control function
            speed = arrSteer[0];
            dir = arrSteer[1];

            //std::cout << "goal angle: " << global_goal_angle << std::endl;
            //std::cout << "filler" << std::endl;

//            if (global_goal_angle > 0.4)
//                dir = -0.4;
//            else if (global_goal_angle < -0.4)
//                dir = 0.4;
//            else
//                dir = arrSteer[1];


            //speed = 0.0;
            //dir = 0.0;

            double xx = (gaz_x_pos*6*1.417 + map_pixel_width/2);
            double yy = (-gaz_y_pos*6*1.417 + map_pixel_height/2);

            double xxx = 0;
            double yyy = 0;

            getParticlesEstimatedPosition(particles, xxx, yyy, estimated_rot);




            if(pose_it > save_pose_every)
            {
                pose_it = 0;

                //void getParticlesEstimatedPosition(std::vector<Particle>& particles, double &x, double &y)

                cv::circle(map_poses, cv::Point2i((int)xx,(int)yy), 4, cv::Scalar(255,0,255), 2, 8);
                cv::circle(map_poses, cv::Point2i((int)xxx,(int)yyy), 2, cv::Scalar(255,0,0), 2, 8);
                //pose_file << "2;\n";
                //std::vector<cv::Point2i> real_pose;
            }

            xxx = (xxx - map_pixel_width/2)/1.417/6;
            yyy = -(yyy - map_pixel_height/2)/1.417/6;
            //std::cout << "Rotation: Estimated: " << estimated_rot << ", Real: " << robot_orientation << std::endl;
            pose_file << gaz_x_pos << ";" << gaz_y_pos<< ";" << xxx << ";" << yyy << ";" << robot_orientation << ";" << estimated_rot << ";\n";

//            if (timestep % 200 == 0)
//            {
//                cv::circle(map_poses, cv::Point2i((int)xx,(int)yy), 4, cv::Scalar(0,128,0), 2, 8);
//                cv::putText(map_poses, std::to_string(timestep/100), cv::Point2i((int)xx,(int)yy-20), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,128,0),3);
//            }


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

//            map_particles = map.clone();
//            for (auto & p : particles)
//            {
//                cv::circle(map_particles, cv::Point2i((int)p.col,(int)p.row), 2, cv::Scalar(0,127,0), 2, 8);
//            }

            if (nearTargetWaypoint(seq_waypoints[index_waypoint], 2) && index_waypoint < seq_waypoints.size())
            {
                index_waypoint++;
                goal_x = (seq_waypoints[index_waypoint].y - 120 / 2)/1.417;
                goal_y = -(seq_waypoints[index_waypoint].x - 80 / 2)/1.417;

            }
            ;
            // " -- Current: x" << waypoints[32].x << " -- y: " << waypoints[32].y <<
            //std::cout << "Goal: x: " << goal_x << " -- y: " << goal_y << " -- near waypoint: " << nearTargetWaypoint(waypoints[32], 5) << std::endl;

            //showImage("Particles", map_particles);


            if (index_waypoint == seq_waypoints.size() - 1)
            {
                cv::imwrite("tracking.png", map_poses);
                cv::imwrite("particles.png", map_particles);
                pose_file.close();
                return 0;
            }
            showImage("Poses", map_poses);

            tick--;
            timestep++;



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
