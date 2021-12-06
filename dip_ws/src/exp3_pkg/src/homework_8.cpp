//
// Created by demphi on 2021/10/26.
//

#include "stdlib.h"
#include "cv.h"
#include "highgui.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "math.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include <algorithm>

#define LINEAR_X 0

using namespace cv;
using namespace std;

Mat src, grey, thresh;
Mat dst_sobel, dst_roberts, dst_prewitt, dst_log, dst_canny;

void Sobel_edge(){
    Mat dx, dy;
//    cv::Size c_size(3, 3);
//    cv::GaussianBlur(src, src, c_size, 1);
    cv::Sobel(src, dx, src.depth(), 1, 0);
    cv::Sobel(src, dy, src.depth(), 0, 1);
    convertScaleAbs(dx, dx);
    convertScaleAbs(dy, dy);
    addWeighted(dx, 0.5, dy, 0.5, 0, dst_sobel);

    imshow("Sobel", dst_sobel);
}

void Roberts_edge(){
    dst_roberts = grey.clone();
    cv::Size c_size(3, 3);
    cv::GaussianBlur(grey, grey, c_size, 1);
    int nRows = dst_roberts.rows;
    int nCols = dst_roberts.cols;
    // calculate gradient
    for(int i=0; i<nRows-1; i++){
        for(int j=0; j<nCols-1; j++){
            int t1 = (grey.at<uchar>(i,j) - grey.at<uchar>(i+1,j+1))*(grey.at<uchar>(i,j) - grey.at<uchar>(i+1,j+1));
            int t2 = (grey.at<uchar>(i+1,j) - grey.at<uchar>(i,j+1))*(grey.at<uchar>(i+1,j) - grey.at<uchar>(i,j+1));
            dst_roberts.at<uchar>(i, j) = (uchar)sqrt(t1+t2);
        }
    }
    imshow("Roberts", dst_roberts);
}

void Prewitt_operator(Mat& get_operator1, Mat& get_operator2){
    //horizon
    get_operator1 = (cv::Mat_<float>(3, 3) << 1, 0, -1, 1, 0, -1, 1, 0, -1 );
    //vertical
    get_operator2 = (cv::Mat_<float>(3, 3) << 1, 1, 1, 0, 0, 0, -1, -1, -1 );

    cv::flip(get_operator1, get_operator1, -1);
    cv::flip(get_operator2, get_operator2, -1);
}

void Prewitt_edge(){
    Mat operator1, operator2;
    Prewitt_operator(operator1, operator2);

    dst_prewitt = src.clone();

    Mat dst_prewitt1, dst_prewitt2;
    //filter
    filter2D(src, dst_prewitt1, src.depth(), operator1);
    filter2D(src, dst_prewitt2, src.depth(), operator2);
    //convert to uint8
    convertScaleAbs(dst_prewitt1, dst_prewitt1);
    convertScaleAbs(dst_prewitt2, dst_prewitt2);

/*    for(int i=0; i<dst_prewitt.rows; i++){
        for(int j=0; j<dst_prewitt.cols; j++){
            dst_prewitt.at<uchar>(i, j) = (uchar)sqrt(dst_prewitt1.at<uchar>(i, j)*dst_prewitt1.at<uchar>(i, j)+dst_prewitt2.at<uchar>(i, j)*dst_prewitt2.at<uchar>(i, j));
        }
    }
    imshow("Prewitt", dst_prewitt);*/

    dst_prewitt = dst_prewitt1+dst_prewitt2;
    imshow("Prewitt", dst_prewitt);

}

void LoG_edge(){
    Laplacian(src, dst_log, src.depth(), 3);
    convertScaleAbs(dst_log, dst_log);

    imshow("LoG", dst_log);
}

void Canny_edge(){
    dst_canny = grey.clone();
    Canny(dst_canny, dst_canny, 10, 50, 3);
    imshow("Canny", dst_canny);
}

int main(int argc, char** argv){
    string img = "/home/demphi/ros/dip_ws/src/exp3_pkg/include/lena.jpeg";
    src = imread(img);
    if(src.empty()){
        ROS_INFO("Frame is empty! Please check!");
        exit(0);
    }
    while(!src.empty()){
        imshow("source picture", src);
        cvtColor(src, grey, COLOR_BGR2GRAY);
        cv::threshold(grey, thresh, 127, 255, THRESH_BINARY);
        Sobel_edge();
        Roberts_edge();
        Prewitt_edge();
        LoG_edge();
        Canny_edge();
        waitKey(10);
    }
    return 0;
}