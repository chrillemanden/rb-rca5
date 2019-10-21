#include <opencv2/opencv.hpp>

int canny_threshold1;
int canny_threshold2;
cv::Mat edges;
cv::Mat img;

void get_frame(cv::Mat& outputImg, int frameNum)
{
    cv::VideoCapture cap("videoOuput.avi");

    if(!cap.isOpened())
    {
        std::cout << "Error opening video stream or file" << std::endl;
    }

    for(int i = 0; i < frameNum; i++)
    {
        cap >> img;
        cv::imshow("video", img);
        cv::waitKey(25);
    }

    outputImg = img.clone();
}

void canny_trackbar_callback(int, void* thresholdSelector)
{

    cv::Canny(img, edges, canny_threshold1, canny_threshold2);
    cv::imshow("Edge Detection", edges);
}

void equalize_luminance(cv::Mat& inputImg, cv::Mat& outputimg)
{
    cv::Mat hls_img;
    cv::cvtColor(inputImg, hls_img, cv::COLOR_BGR2HLS);

    std::vector<cv::Mat> channels(hls_img.channels());
    cv::split(hls_img, channels);
    cv::equalizeHist(channels[1], channels[1]);
    cv::merge(channels, hls_img);
    cv::cvtColor(hls_img, outputimg, cv::COLOR_HLS2BGR);
}

void pre_processing(cv::Mat& inputImg, cv::Mat& outputImg)
{
    cv::Mat temp_img;
    equalize_luminance(inputImg, temp_img);
    cv::GaussianBlur(temp_img, temp_img, cv::Size(9,9), 2, 2);
    cv::medianBlur(temp_img, temp_img, 3);
    outputImg = temp_img.clone();
}

std::vector<cv::Vec3f> detect_marbels(cv::Mat& inputImg)
{
    std::vector<cv::Vec3f> circles;

    cv::Mat test_img;
    cv::cvtColor(inputImg, test_img, cv::COLOR_BGR2GRAY);

    cv::HoughCircles(test_img, circles,cv::HOUGH_GRADIENT, 1, inputImg.rows/12, 170, 24, 0, 0);

    for(int i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        //Draw outline of marble
         cv::circle(inputImg, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
    }

    return circles;
}


int main(int argc, char *argv[])
{
    cv::VideoCapture cap("videoOuput.avi");
    cv::VideoWriter video("processed_camera_output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(320, 240));

    if(cap.isOpened() == false)
    {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    cv::Mat camera_input;
    cv::namedWindow("Display window 1", cv::WINDOW_AUTOSIZE);

    //for(;;) //use for the complete film
    for(int i = 0; i < 150; i++) //use for only the first couple of frames
    {
        cap >> camera_input;

        if(!cap.isOpened())
        {
            std::cout << "empty" << std::endl;
            break;
        }

        pre_processing(camera_input, camera_input);
        detect_marbels(camera_input);

        cv::imshow("Display window 1", camera_input);
        video.write(camera_input);
        cv::waitKey(25);
    }
    cap.release();
    video.release();
    cv::waitKey(0);
    return 1;
}
