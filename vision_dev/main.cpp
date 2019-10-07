#include <opencv2/opencv.hpp>

void get_frame(cv::Mat& outputImg, int frameNum)
{
    cv::VideoCapture cap("videoOuput.avi");

    if(!cap.isOpened())
    {
        std::cout << "Error opening video stream or file" << std::endl;
    }

    cv::Mat img;

    for(int i = 0; i < frameNum; i++)
    {
        cap >>img;
    }

    outputImg = img.clone();
}


int main(int argc, char *argv[])
{
    cv::Mat img;
    get_frame(img, 1);

    cv::namedWindow("Display window 1", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display window 1", img);

    cv::waitKey(0);
    return 1;
}
