#include <opencv2/opencv.hpp>

int canny_threshold1;
int canny_threshold2;
cv::Mat edges;
cv::Mat img;

//values for tuning detect_contour2()
int min_area = 50;


/*void get_frame(cv::Mat& outputImg, int frameNum)
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
}*/

/*void canny_trackbar_callback(int, void* thresholdSelector)
{

    cv::Canny(img, edges, canny_threshold1, canny_threshold2);
    cv::imshow("Edge Detection", edges);
}*/

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
    cv::Mat test_img = inputImg;
    //Detect marbels using houghcircles()
    //cv::cvtColor(inputImg, test_img, cv::COLOR_BGR2GRAY);
    // cv::HoughCircles(test_img, circles,cv::HOUGH_GRADIENT, 1, inputImg.rows/12, 170, 24, 0, 0);
    cv::HoughCircles(test_img, circles,cv::HOUGH_GRADIENT, 1, inputImg.rows/12, 50, 24, 0, 0);

    cv::Mat bin_hough = cv::Mat::zeros(240, 320, CV_8U);

    for(unsigned int i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        //Draw outline of marble
         cv::circle(bin_hough, center, radius, cv::Scalar(255,255,255), -1, 8, 0);
    }

    inputImg = bin_hough.clone();
    return circles;
}

void hough_circle_detection(cv::Mat& input_img, cv::Mat& output_img, std::vector<cv::Rect>& bounding_box_vector)
{
    //Black image for displaying the contours found by the hough circle algorithm
    cv::Mat contour_img = cv::Mat::zeros(240, 320, CV_8U);
    cv::Mat gray_img;

    //clear the input vector to ensure it only contains ole Rects
    bounding_box_vector.clear();

    //Run the hough circle transformation
    std::vector<cv::Vec3f> detected_circles;
    cv::cvtColor(input_img, gray_img, cv::COLOR_BGR2GRAY);
    cv::HoughCircles(gray_img, detected_circles, cv::HOUGH_GRADIENT, 1, input_img.rows/12, 170, 24, 0, 0);


    //Draw the found circles
    for(unsigned int i = 0; i < detected_circles.size(); i++)
    {
        cv::Point center(cvRound(detected_circles[i][0]), cvRound(detected_circles[i][1]));
        int radius = cvRound(detected_circles[i][2]);

        //Draw the circles on the contour image
        cv::circle(contour_img, center, radius, cv::Scalar(255,255,255), -1, 8, 0);
    }

    //Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarcy;

    cv::findContours(contour_img, contours, hierarcy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    //Comvert contours to bounding boxes and store them on bounding_box_vector
    for(unsigned int i = 0; i < contours.size(); i++)
    {
        bounding_box_vector.push_back(cv::boundingRect(contours[i]));
    }

    output_img = contour_img.clone();



}

void contour_detection2(cv::Mat& input_img, cv::Mat& output_img, std::vector<cv::Rect>& bounding_box_vector)
{
    cv::Mat img_gray;
    cv::Mat img_bin;
    cv::Mat img_contour = cv::Mat::zeros(240, 320, CV_8U); //Initialize a black image for displaying relevant contours

    //Clear the input vector to ensure it only contains old Rects
    bounding_box_vector.clear();

    // Conver image to greyscale
    cv::cvtColor(input_img, img_gray, cv::COLOR_BGR2GRAY);

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
    for(unsigned int i = 0; i < contours.size(); i++)
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
        if(isgreaterequal(area_contour, area_bounding_rect * 0.85) || isgreaterequal(area_bounding_rect*0.68, area_contour))
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

        //Draw the hull to minimize holes in the contour
        contours[i] = hull;

        //Draw the contour
        cv::Scalar colour(0xFF, 0xFF, 0xFF );
        cv::drawContours(img_contour, contours, i, colour, CV_FILLED, 8, hierarcy);

        std::cout << "pushed to vector" << std::endl;
        //Save the Rect on the input vector
        bounding_box_vector.push_back(bounding_rect);
        std::cout << bounding_box_vector.size() << std::endl;

    }

    output_img = img_contour.clone();
}


std::vector<cv::Rect> filter_BB(std::vector<cv::Rect> cur_BB_hough, std::vector<cv::Rect> prev_BB_hough, std::vector<cv::Rect> cur_BB_contour, std::vector<cv::Rect> prev_BB_contour)
{
    std::cout << "started filter_BB" << std::endl;
    std::vector<std::vector<cv::Rect>> all_BB = {cur_BB_hough,  cur_BB_contour, prev_BB_hough, prev_BB_contour};
    std::vector<cv::Rect> BB_of_interest;

    bool stop = false;

    for(unsigned int i = 0; i < 4; i++)
    {
        //std::cout << "for loop 1" << std::endl;
        for(unsigned int u = 0; u < all_BB[i].size(); u++)
        {
            //std::cout << "for loop 2" << std::endl;
            stop = false;
            cv::Rect test_rect1 = all_BB[i][u];


            for(unsigned int j = 0; j < 4 && !stop; j++)
            {
                //std::cout << "for loop 3" << std::endl;
                if(j == i)
                {
                    continue;
                }

                for(unsigned int k = 0; k < all_BB[j].size() && !stop; k++)
                {
                    //std::cout << "for loop 4" << std::endl;
                    cv::Rect test_rect2 = all_BB[j][k];
                    double intersection_area = (test_rect1 & test_rect2).area();

                    //Calculate the intersection of the two rects
                    if(isgreaterequal(intersection_area, test_rect1.area() * 0.80))
                    //if(intersection_area > test_rect1.area() * 0.80)
                    {
                      BB_of_interest.push_back(test_rect1);
                      stop = true;
                    }
                }
            }
        }
    }
    std::cout << "finished filter_BB" << std::endl;
    return BB_of_interest;
}


void draw_bounding_boxes(std::vector<cv::Rect> bounding_boxes, cv::Mat output_img, cv::Scalar color)
{
    //cv::Scalar green_color = cv::Scalar(0, 255, 0);

    for(unsigned int i = 0; i < bounding_boxes.size(); i++)
    {
        cv::rectangle(output_img, bounding_boxes[i].tl(), bounding_boxes[i].br(), color, 2);
    }
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
    for(unsigned int i = 0; i < contours.size(); i=i)
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

        for(unsigned int i = 0; i < contours.size(); i++)
        {
            cv::Scalar colour(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF );
            cv::drawContours(mat_contours, contours, i, colour, CV_FILLED, 8, hierarcy);
        }
    }

    img = mat_contours.clone();
}

std::vector<cv::Rect> merge_overlapping_BB(std::vector<cv::Rect> list_of_BB)
{
    std::vector<cv::Rect> BB_merged;

    cv::Mat bin_img = cv::Mat::zeros(240, 320, CV_8U);

    for(unsigned int i = 0; i < list_of_BB.size(); i++)
    {
        cv::rectangle(bin_img, list_of_BB[i].tl(), list_of_BB[i].br(), cv::Scalar(255, 255, 255), CV_FILLED);
    }

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarcy;
    cv::findContours(bin_img, contours, hierarcy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    for(unsigned int i = 0; i < contours.size(); i++)
    {
        BB_merged.push_back(cv::boundingRect(contours[i]));
    }

    return BB_merged;
}


void merge_bin_images(cv::Mat input_img1, cv::Mat input_img2, cv::Mat input_img3, cv::Mat input_img4, cv::Mat& output_img)
{
   cv::Mat xor1;
   cv::Mat xor2;
   cv::Mat xor3;
   cv::Mat xor4;
   cv::Mat xor5;
   cv::Mat xor6;
   cv::Mat res;

   cv::bitwise_and(input_img1, input_img2, xor1);
   cv::bitwise_and(input_img1, input_img3, xor2);
   cv::bitwise_and(input_img1, input_img4, xor3);
   cv::bitwise_and(input_img2, input_img3, xor4);
   cv::bitwise_and(input_img2, input_img4, xor5);
   cv::bitwise_and(input_img3, input_img4, xor6);

   cv::bitwise_or(xor1, xor2, res);
   cv::bitwise_or(res, xor3, res);
   cv::bitwise_or(res, xor4, res);
   cv::bitwise_or(res, xor5, res);
   cv::bitwise_or(res, xor6, res);

    output_img = res.clone();

}


std::vector<cv::Rect> get_BB_from_bin(cv::Mat bin_image)
{
    std::vector<cv::Rect> BB_vec;
    //Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarcy;
    cv::findContours(bin_image, contours, hierarcy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    for(unsigned int i = 0; i < contours.size(); i++)
    {
        BB_vec.push_back(cv::boundingRect(contours[i]));
    }

    return BB_vec;

}

int main(int argc, char *argv[])
{
    //Load video
    cv::VideoCapture cap("videoOuput.avi");
    cv::VideoWriter video("processed_camera_output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(320, 240));

    if(cap.isOpened() == false)
    {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    //Defining image containers used in the main loop
    cv::Mat camera_input; //Container to load individual frames into
    cv::Mat preproc_img; //Container for storing the preprocessed img
    cv::Mat cont_detect_img; //Container for storing the binary outut image from the contour detection algorithm
    cv::Mat hough_detect_img;
    cv::Mat prev_cont_detect_img = cv::Mat::zeros(240, 320, CV_8U);;
    cv::Mat prev_hough_detect_img = cv::Mat::zeros(240, 320, CV_8U);;
    cv::Mat merged_detect;

    //Defining different output windows
    cv::namedWindow("Contour Detection Output", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Hough Detection Output", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Previous Contour Detection Output", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Previous Hough Detection Output", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Merged Detection Output", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Raw Image Input", cv::WINDOW_AUTOSIZE);

    std::vector<cv::Rect> BB_contour_detection; //vector for storing bounding boxes found by the contour detection algorithm
    std::vector<cv::Rect> BB_hough_detection; //vector for storing bounding boxes found by the hough circle algorithm
    std::vector<cv::Rect> prev_BB_contour_detection;
    std::vector<cv::Rect> prev_BB_hough_detection;
    std::vector<cv::Rect> BB_of_interest;
    std::vector<cv::Rect> BB_of_interest_merged;
    std::vector<cv::Rect> BB_merged_overlapping;

    //Colours used for bounding boxes
    cv::Scalar green_color = cv::Scalar(0, 255, 0);
    cv::Scalar red_color = cv::Scalar(0, 0, 255);
    cv::Scalar blue_color = cv::Scalar(255, 0, 0);

    //for(;;) //use for displaying the complete film
    for(int i = 0; i < 150; i++) //use for only getting the first part of the frames
    {
        cap >> camera_input; //Load individual frame

        if(!cap.isOpened())
        {
            std::cout << "empty" << std::endl;
            break;
        }

        cv::Mat raw_img = camera_input;

        //Image pipeline
        pre_processing(camera_input, preproc_img);
        contour_detection2(preproc_img, cont_detect_img, BB_contour_detection);
        hough_circle_detection(preproc_img, hough_detect_img, BB_hough_detection);
        merge_bin_images(cont_detect_img, prev_cont_detect_img, hough_detect_img, prev_hough_detect_img, merged_detect);
        BB_of_interest_merged = get_BB_from_bin(merged_detect);
        BB_merged_overlapping = merge_overlapping_BB(BB_of_interest_merged);

        //BB_of_interest = filter_BB(BB_hough_detection, prev_BB_hough_detection, BB_contour_detection, prev_BB_contour_detection);


        //Display the found BB in the raw image input
        //draw_bounding_boxes(BB_hough_detection, raw_img, red_color);
        //draw_bounding_boxes(BB_of_interest, raw_img, blue_color);
        //draw_bounding_boxes(BB_contour_detection, raw_img, green_color);
        //draw_bounding_boxes(BB_of_interest_merged, raw_img, green_color);
        draw_bounding_boxes(BB_merged_overlapping, raw_img, green_color);

        //Display images
        cv::imshow("Raw Image Input", camera_input);
        cv::imshow("Contour Detection Output", cont_detect_img);
        cv::imshow("Hough Detection Output", hough_detect_img);
        cv::imshow("Previous Contour Detection Output", prev_cont_detect_img);
        cv::imshow("Previous Hough Detection Output", prev_hough_detect_img);
        cv::imshow("Merged Detection Output", merged_detect);


        //Update data from previous images;
        prev_cont_detect_img = cont_detect_img.clone();
        prev_hough_detect_img = hough_detect_img.clone();

        prev_BB_contour_detection = BB_contour_detection;
        prev_BB_hough_detection = BB_hough_detection;

        cv::waitKey(25);
    }


    cap.release();
    video.release();

    cv::waitKey(0);

    return 1;
}
