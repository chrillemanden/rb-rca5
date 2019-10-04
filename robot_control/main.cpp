#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include "fl/Headers.h"
#include <boost/math/constants/constants.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>

static boost::mutex mutex;
int tick = 0;
const double pi = boost::math::constants::pi<double>();
double global_minDist;
double global_angle;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

  for (int i = 0; i < _msg->pose_size(); i++) {
    if (_msg->pose(i).name() == "pioneer2dx") {

      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
                << _msg->pose(i).position().x() << std::setw(6)
                << _msg->pose(i).position().y() << std::setw(6)
                << _msg->pose(i).position().z() << std::setw(6)
                << _msg->pose(i).orientation().w() << std::setw(6)
                << _msg->pose(i).orientation().x() << std::setw(6)
                << _msg->pose(i).orientation().y() << std::setw(6)
                << _msg->pose(i).orientation().z() << std::endl;
    }
  }
}

void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, CV_RGB2BGR);

  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}

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
    double min_dist_angle = -0.5 * pi + angle_increment * min_dist_index;
    //std::cout << "Angle of min dist in half-circle is: " << min_dist_angle << std::endl;
    global_minDist = min_dist*4;
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

int main(int _argc, char **_argv) {
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

  fl::Engine* engine = fl::FllImporter().fromFile("ObstacleAvoidance.fll");
  std::string status;
   engine->isReady(&status);
   std::cout << "test" << status << std::endl;

   if (not engine->isReady(&status))
     {
       std::cout << "not found" << std::endl;
       throw fl::Exception("[engine error] engine is not ready:n" + status, FL_AT);
     }

   fl::InputVariable* obstacle_distance = engine->getInputVariable("obstacle_distance");
   fl::InputVariable* obstacle_angle = engine->getInputVariable("obstacle_angle");
   fl::OutputVariable* steer = engine->getOutputVariable("mSteer");



  float speed = 0.25;
  float dir = 0.0;

  // Loop
    while (true)
    {
        gazebo::common::Time::MSleep(10);

	mutex.lock();
    	int key = cv::waitKey(1);
    	mutex.unlock();
        
	if(tick > 0)
        {
            fl::scalar distance = global_minDist;
            fl::scalar angle = global_angle;
            obstacle_angle->setValue(angle);
            obstacle_distance->setValue(distance);
            engine->process();
            dir = steer->getValue();
            tick--;
            std::cout << "global_minDist: " << distance << std::endl;
            std::cout << "dir: " << dir << std::endl;
        }



    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(dir));

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);
    }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
