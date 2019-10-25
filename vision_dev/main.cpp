#include <opencv2/opencv.hpp>

int canny_threshold1;
int canny_threshold2;
cv::Mat edges;
cv::Mat img;

//values for tuning detect_contour2()
int min_area = 50;


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

//std::vector<cv::Vec3f> reduce_circle_vec(std::vector<Vec3f> initial_list)
//{
//    std::vector<cv::Vec3f>::iterator last_it;
//    for(std::vector<cv::Vec3f>::iterator first_it = initial_list.begin(); first_it != initial_list.end(); first_it++)
//    {
//    }
//}

std::vector<cv::Vec3f> detect_marbels(cv::Mat& inputImg)
{
    std::vector<cv::Vec3f> circles;
    cv::Mat test_img = inputImg;
    //Detect marbels using houghcircles()
    //cv::cvtColor(inputImg, test_img, cv::COLOR_BGR2GRAY);
    // cv::HoughCircles(test_img, circles,cv::HOUGH_GRADIENT, 1, inputImg.rows/12, 170, 24, 0, 0);
    cv::HoughCircles(test_img, circles,cv::HOUGH_GRADIENT, 1, inputImg.rows/12, 50, 24, 0, 0);

    cv::Mat bin_hough = cv::Mat::zeros(240, 320, CV_8U);

    for(int i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        //Draw outline of marble
         cv::circle(bin_hough, center, radius, cv::Scalar(255,255,255), -1, 8, 0);
    }

    inputImg = bin_hough.clone();
    return circles;
}

void contour_detection2(cv::Mat& img)
{
    cv::Mat img_gray;
    cv::Mat img_bin;
    cv::Mat img_contour = cv::Mat::zeros(240, 320, CV_8U); //Initialize a black image for displaying relevant contours

    // Conver image to greyscale
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    //use adaptive thresholding to convert the image to binary
    cv::adaptiveThreshold(img_gray, img_bin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 101,1);

    //Erode and dialate to prepare for contour detetction
    //Kernels used
    cv::Mat structuring_element3(3, 3, CV_8U, cv::Scalar(1));
    cv::Mat structuring_element5(5, 5, CV_8U, cv::Scalar(1));

    for(int i = 0; i < 3; i++)
    {
        cv::erode(img_bin, img_bin, structuring_element5);
    }
    for(int i = 0; i < 8; i++)
    {
        cv::dilate(img_bin, img_bin, structuring_element3);
    }

    //Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarcy;
    cv::findContours(img_bin, contours, hierarcy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    //Loop for detecting if a contour is of interest
    for(int i = 0; i < contours.size(); i++)
    {
        //Calculating the area of the contour
        double area_contour = cv::contourArea(contours[i]);
        std::cout << "Contour area - " << i << " : " << area_contour << std::endl;

        //If the contour's area is smaller than minimum area, the contour is not relevant.
        if(isgreater(min_area, area_contour))
        {
            std::cout << "contour thrown away" << std::endl;
            continue;
        }

        //Calculate the bounding box around the contour
        cv::Rect bounding_rect = cv::boundingRect(contours[i]);
        double area_bounding_rect = bounding_rect.height * bounding_rect.width;

        //if the bounding box' width much greater than the width, the contour cannot be a circle - the contour is not relevant.
        if(isgreaterequal(bounding_rect.width, bounding_rect.height*1.2))
        {
            std::cout << "contour thrown away" << std::endl;
            continue;
        }

        //If the area of the bounding box and the countour itself is more or less equal to each other, the
        //the contour will be a sqaure - the contour is not relevant.
        std::cout << "area contour : " << area_contour <<  " Area bouding box: " << area_bounding_rect << std::endl;
        if(isgreaterequal(area_contour, area_bounding_rect * 0.85) || isgreaterequal(area_bounding_rect*0.70, area_contour))
        {
            std::cout << "contour thrown away" << std::endl;
            continue;
        }

        //Fit at rotated box around the contour
        std::vector<cv::Point> hull;
        cv::convexHull(contours[i], hull);
        cv::RotatedRect bounding_rect_rotated = cv::minAreaRect(hull);
        cv::Size2f size_bounding_box_rotated = bounding_rect_rotated.size;
        double area_bounding_rect_rotated = size_bounding_box_rotated.area();

        std::cout << "Bounding Box rotated - " << i << " : "<< area_bounding_rect_rotated << std::endl;
        if(isgreaterequal(area_contour, area_bounding_rect_rotated * 0.85) || isgreaterequal(area_bounding_rect_rotated*0.70, area_contour))
        {
            std::cout << "contour thrown away" << std::endl;
            continue;
        }

        std::cout << "Contour is drawn " << std::endl;
        //Draw the contour
        cv::Scalar colour(0xFF, 0xFF, 0xFF );
        cv::drawContours(img_contour, contours, i, colour, CV_FILLED, 8, hierarcy);

    }



    img = img_contour.clone();
}

void contour_detection(cv::Mat& img)
{
    cv::Mat img_gray;
    cv::Mat img_bin;

    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    cv::adaptiveThreshold(img_gray, img_bin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 101,1);
    cv::Mat structuring_element3(3, 3, CV_8U, cv::Scalar(1));
    cv::Mat structuring_element5(5, 5, CV_8U, cv::Scalar(1));
    //cv::erode(img_bin, img_bin, structuring_element5);
    for(int i = 0; i < 3; i++)
    {
        cv::erode(img_bin, img_bin, structuring_element5);
    }
    for(int i = 0; i < 8; i++)
    {
        cv::dilate(img_bin, img_bin, structuring_element3);
    }

   std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarcy;
    cv::findContours(img_bin, contours, hierarcy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    std::cout << "number of contours" << contours.size() << std::endl;
    for(int i = 0; i < contours.size(); i=i)
    {
        double area_contour = cv::contourArea(contours[i]);
        std::cout << "Area Contour - " << i << " : " << area_contour << std::endl;
        if(isgreater(min_area, area_contour))
        {
            break;
        }

        std::vector<cv::Point> hull;
        cv::convexHull(contours[i], hull);
        cv::RotatedRect boundingBox = cv::minAreaRect(hull);
        //cv::Rect boundingBox = cv::boundingRect(hull);
        cv::Size2f size_BOX = boundingBox.size;

        double area_bounding_box = size_BOX.area();
        std::cout << "Bounding Box - " << i << " : "<< area_bounding_box << std::endl;


        double diff_area = area_bounding_box - area_contour;
        std::cout << "Diff area" << diff_area << std::endl;
        std::cout << (1.0-0.885)*area_bounding_box << std::endl;
        std::cout << (1.0-0.685)*area_bounding_box << std::endl;
        //A perfect circle will cover 0.785 percent of the smallest square the circle fits in.
        //if( ((1.0-0.885)*area_bounding_box) < diff_area && diff_area < ((1.0-0.685)*area_bounding_box));
        if(isgreaterequal(diff_area, (1.0-0.835)*area_bounding_box) && isgreaterequal((1.0-0.735)*area_bounding_box, diff_area))
        {
            std::cout << "Entered if statement" << std::endl;
            contours[i] = hull;
            //std::cout << contourArea(contours[i]) << std::endl;
            //cv::Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF );
            //cv::drawContours(img_bin, contours, i, colour, CV_FILLED, 8, hierarcy);
            i++;
        }
        else
        {
            contours.erase(contours.begin() + i);
            hierarcy.erase(hierarcy.begin() + i);
        }

    }
    cv::Mat mat_contours = cv::Mat::zeros(240, 320, CV_8U);

    if(contours.size() > 0)
    {

        for(int i = 0; i < contours.size(); i++)
        {
            cv::Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF );
            cv::drawContours(mat_contours, contours, i, colour, CV_FILLED, 8, hierarcy);
        }
    }

    img = mat_contours.clone();
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
        cv::Mat raw_img = camera_input;
        pre_processing(camera_input, camera_input);
        contour_detection2(camera_input);

        //pre_processing(camera_input, camera_input);
        //detect_marbels(camera_input);
        //cv::Mat camera_bin;
        //cv::cvtColor(camera_input, camera_bin, cv::COLOR_BGR2GRAY);
        //cv::adaptiveThreshold(camera_bin, camera_bin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 121, -15);
        //cv::imshow("Display window 1", camera_bin);
        cv::imshow("Displa window 2", camera_input);
        cv::imshow("Display window 1", raw_img);
        //video.write(camera_input);
        cv::waitKey(25);
    }
    cap.release();
    video.release();



    //cv::imshow("test", bin_hough);
    cv::waitKey(0);
    return 1;
}
