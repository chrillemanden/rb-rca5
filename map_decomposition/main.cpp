#include <iostream>
#include <opencv2/opencv.hpp>
#include <getCorners.cpp>
#include <Kmeans_clustering.cpp>

void expand_centerPoint(cv::Mat& img_roi, cv::Vec3b color)
{
        int x_center =cvRound(img_roi.cols/2.0);
        int y_center = cvRound(img_roi.rows/2.0);

        cv::Vec3b center_pixel = img_roi.at<cv::Vec3b>(y_center, x_center);

        bool wall_enc_up = false;
        bool wall_enc_down = false;
        bool wall_enc_left = false;
        bool wall_enc_right = false;

        if(center_pixel != cv::Vec3b(0,0,0))
        {


                for(int i = 0; (y_center - i) >= 0 ||(y_center + i) < img_roi.rows; i++)
                {
                    if((i + y_center) <= img_roi.rows -1 && wall_enc_down == false)
                    {
                        cv::Vec3b test_pixel = img_roi.at<cv::Vec3b>((i + y_center), x_center);
                        if(test_pixel == cv::Vec3b(0,0,0) || wall_enc_down == true)
                        {
                            wall_enc_down = true;
                        }
                        else
                        {
                            img_roi.at<cv::Vec3b>((i + y_center), x_center) = color;
                        }

                    }

                    if((y_center - i) >= 0 && wall_enc_up == false)
                    {
                        cv::Vec3b test_pixel = img_roi.at<cv::Vec3b>((y_center - i), x_center);
                        if(test_pixel == cv::Vec3b(0,0,0) || wall_enc_up == true)
                        {
                            wall_enc_up = true;
                        }
                        else
                        {
                                img_roi.at<cv::Vec3b>((y_center - i), x_center) = color;
                        }



                    }
                }

                img_roi.at<cv::Vec3b>(y_center, x_center) = cv::Vec3b(255, 0, 0);
                cv::waitKey(0);
                cv::namedWindow("test", CV_WINDOW_NORMAL);
                cv::imshow("test", img_roi);


        }

}

std::vector<std::vector<int>> split_XY(std::vector<cv::Point2i> point_list)
{

    std::vector<std::vector<int>> XY_list(2, std::vector<int>(0, 0));

    std::cout << "point list size: " << point_list.size() << std::endl;
    for(unsigned int i = 0; i < point_list.size(); i++)
    {

        XY_list[0].push_back(point_list[i].y);
        XY_list[1].push_back(point_list[i].x); //X and Y apear to be switched


        std::cout << "m- " << "X: " << point_list[i].x << " Y: " << point_list[i].y << std::endl;
    }

    std::cout << "point list size: " << point_list.size() << std::endl;
    return XY_list;

}




int main()
{
    cv::Mat maze = cv::imread("floor_plan.png", CV_LOAD_IMAGE_GRAYSCALE);
    cv::namedWindow("maze", CV_WINDOW_NORMAL);
    cv::imshow("maze", maze);
    std::cout << maze.cols << " - " << maze.rows << std::endl;

    std::vector<cv::Point2i> corner_list;

    getCorners(maze, corner_list);

    //print corner list
    for(int i = 0; i < corner_list.size(); i++)
    {
        std::cout << corner_list[i] << endl;
    }

    std::vector<vector<int>> XY_list = split_XY(corner_list);

    for(int i = 0; i < XY_list[0].size(); i++)
    {
        std::cout << "X: " << XY_list[0][i] << " - Y: " << XY_list[1][i] << std::endl;
    }

    vector<float> X_clusters = KMeansClustering(XY_list[0], 8, 10);
    vector<float> Y_clusters = KMeansClustering(XY_list[1], 6, 10);

    cv::Mat maze_color;
    cv::cvtColor(maze, maze_color, CV_GRAY2BGR);



    //Draw vertical lines
    //cv::Mat maze_color;
    //cv::cvtColor(maze, maze_color, CV_GRAY2BGR);
    //std::cout << X_clusters.size() << std::endl;

    //Draw horisontal lines
    for(int i = 0; i < Y_clusters.size(); i++)
    {
        cv::line(maze_color, cv::Point(0, cvRound(cvRound(Y_clusters[i]))), cv::Point(maze.cols, cvRound(cvRound(Y_clusters[i]))), cv::Scalar(0, 255,0));
        std::cout << "Drawing line" << i << " - " << Y_clusters[i] << std::endl;
    }

    //Draw vertical lines
    for(int i = 0; i < X_clusters.size();i++)
    {
        cv::line(maze_color, cv::Point(cvRound(X_clusters[i]), 0), cv::Point(cvRound(X_clusters[i]), maze.rows), cv::Scalar(0, 255,0));
        std::cout << "Drawing line" << i << " - " << Y_clusters[i] << std::endl;
    }

     cv::namedWindow("test 5", CV_WINDOW_NORMAL);
    //insert points
    for(int i = 0; i < Y_clusters.size()-1; i++)
    {
        for(int j = 0; j < X_clusters.size()-1; j++)
        {

            float y_pos = cvRound((Y_clusters[i+1]-Y_clusters[i])/2.0+Y_clusters[i]);
            float x_pos = cvRound((X_clusters[j+1]-X_clusters[j])/2.0+X_clusters[j]);

            cv::Rect roi = cv::Rect(std::floor(X_clusters[j]), std::floor(Y_clusters[i]), std::floor(X_clusters[j+1]-X_clusters[j])+1, std::floor(Y_clusters[i+1]-Y_clusters[i])+1);

            cv::Mat mat_roi = maze_color(roi);
            expand_centerPoint(mat_roi, cv::Vec3b(255, 255, 0));

            //
            //mat_roi.at<uchar>(0,0) = 100;
            //
            //cv::imshow("test 5", mat_roi);
            cv::imshow("maze", maze_color);
            cv::waitKey(0);


            cv::circle(maze_color, cv::Point(x_pos, y_pos), 1, cv::Scalar(0,0,255), -1);
        }
    }


    cv::namedWindow("Maze with grid", CV_WINDOW_NORMAL);
    cv::imshow("Maze with grid", maze_color);

    std::cout << maze.cols << maze.rows << std::endl;

    cv::waitKey(0);

    return 0;
}
