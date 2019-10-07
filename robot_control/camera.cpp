#include <opencv2/opencv.hpp>

#include "camera.h"

static boost::mutex mutex;
cv::VideoWriter video;


void init_video_capture()
{
    video = cv::VideoWriter("videoOuput.avi",CV_FOURCC('M','J','P','G'),10, cv::Size(320,240));

}

void destroy_video_capture()
{
    video.release();
}

void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  std::cout << "width: " << width << " height: " << height << std::endl;
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, CV_RGB2BGR);

  //video.write(im);
  mutex.lock();
  cv::imshow("camera", im);
  mutex.unlock();
}
