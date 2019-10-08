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
    cv::GaussianBlur(temp_img, temp_img, cv::Size(5,5), 1.5);
    cv::medianBlur(temp_img, temp_img, 3);
    outputImg = temp_img.clone();
}

std::vector<cv::Vec3f> detect_marbels(cv::Mat& inputImg)
{
    std::vector<cv::Vec3f> circles;

    cv::Mat test_img;
    cv::cvtColor(inputImg, test_img, cv::COLOR_BGR2GRAY);

    cv::HoughCircles(test_img, circles,cv::HOUGH_GRADIENT, 1, inputImg.rows/8, 180, 24, 0, 0);

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


    get_frame(img, 10);
    //cv::cvtColor(img, img, cv::COLOR_BGR2HLS);
    //Display raw image
    pre_processing(img, img);
    detect_marbels(img);

    cv::namedWindow("Display window 1", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display window 1", img);


    //Display edge detected image
    //cv::namedWindow("Edge Detection", cv::WINDOW_AUTOSIZE);

    //char TrackBarName1[10] = "thres 1";
    //char TrackBarName2[10] = "thres 2";
    //cv::createTrackbar(TrackBarName1, "Edge Detection", &canny_threshold1, 255, canny_trackbar_callback );
    //cv::createTrackbar(TrackBarName2, "Edge Detection", &canny_threshold2, 255, canny_trackbar_callback );

    //std::vector<cv::Vec3f> circles;
    //cv::Mat img_gray;
    //cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    //cv::imwrite("video_ouput_img.jpg", img_gray);
    //cv::HoughCircles(img_gray, )


    cv::waitKey(0);
    return 1;
}
