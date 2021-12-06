//
// Created by demphi on 2021/10/16.
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
#include "std_msgs/Float64MultiArray.h"

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include <algorithm>

#define LINEAR_X 0
#define IF_USE_CAMERA 0

#define MAX_SIZE 255
#define MAX_H_SIZE 180

using namespace cv;
using namespace std;

std_msgs::Float64MultiArray data;

Mat src, grey, thresh;
Mat hsv, hsv_split[3];
vector<Mat> hsv_channels;
vector<int> color_hist(7);
vector<int> percentage(7);

int H_min = 0, H_max = 10;
int S_min = 43, S_max = 255;
int V_min = 46, V_max = 255;

struct Color_Range{
    int red_min[2] = {0, 156};
    int red_max[2] = {10, 180};
    int orange_min = 11;
    int orange_max = 25;
    int yellow_min = 26;
    int yellow_max = 40;
    int green_min = 41;
    int green_max = 64;
    int cyan_min = 65;
    int cyan_max = 99;
    int blue_min = 100;
    int blue_max = 124;
    int purple_min = 125;
    int purple_max = 155;
};

enum DisColor{
    rec_red,
    rec_orange,
    rec_yellow,
    rec_green,
    rec_cyan,
    rec_blue,
    rec_purple
};

void RGB2HSV(){
    Size t_size = {3, 3};
    cv::GaussianBlur(src, src, t_size, 1.5);
    hsv = src.clone();
    cv::cvtColor(src, hsv, COLOR_BGR2HSV);
    split(hsv, hsv_channels);
    hsv_split[0] = hsv_channels.at(0);
    hsv_split[1] = hsv_channels.at(1);
    hsv_split[2] = hsv_channels.at(2);
    /*imshow("H Channel", hsv_split[0]);
    imshow("S Channel", hsv_split[1]);
    imshow("V Channel", hsv_split[2]);*/

}

void deterCallback(int, void*){
    Mat temp = hsv_split[0].clone();
/*    for (int i=0; i<hsv_split[0].rows; i++) {
        for(int j=0; j<hsv_split[0].cols; j++){
            if(hsv_split[0].at<uchar>(i,j)<H_max && hsv_split[0].at<uchar>(i,j)>H_min){
                temp.at<uchar>(i,j) = 255;
            } else{
                temp.at<uchar>(i,j) = 0;
            }
        }
    }*/
    threshold(hsv_split[0], temp, H_max, MAX_SIZE, THRESH_TOZERO_INV);
    threshold(temp, temp, H_min, MAX_SIZE, THRESH_TOZERO);
    threshold(temp, temp, H_min, MAX_SIZE, THRESH_BINARY);
    imshow("Binary H picture", temp);
}

void deterCallback_1(int, void*){
    Mat temp = hsv_split[0].clone();
/*    for (int i=0; i<hsv_split[0].rows; i++) {
        for(int j=0; j<hsv_split[0].cols; j++){
            if(hsv_split[0].at<uchar>(i,j)<H_max && hsv_split[0].at<uchar>(i,j)>H_min){
                temp.at<uchar>(i,j) = 255;
            } else{
                temp.at<uchar>(i,j) = 0;
            }
        }
    }*/

    threshold(hsv_split[0], temp, H_max, MAX_SIZE, THRESH_TOZERO_INV);
    threshold(temp, temp, H_min, MAX_SIZE, THRESH_TOZERO);
    threshold(temp, temp, H_min, MAX_SIZE, THRESH_BINARY);

    imshow("Binary H picture", temp);
}

void deterCallback_S(int, void*){
    Mat temp = hsv_split[1].clone();
    threshold(hsv_split[1], temp, S_max, MAX_SIZE, THRESH_TOZERO_INV);
    threshold(temp, temp, S_min, MAX_SIZE, THRESH_TOZERO);
    threshold(temp, temp, S_min, MAX_SIZE, THRESH_BINARY);
    imshow("Binary S picture", temp);
}

void deterCallback_s(int, void*){
    Mat temp = hsv_split[1].clone();
    threshold(hsv_split[1], temp, S_max, MAX_SIZE, THRESH_TOZERO_INV);
    threshold(temp, temp, S_min, MAX_SIZE, THRESH_TOZERO);
    threshold(temp, temp, S_min, MAX_SIZE, THRESH_BINARY);
    imshow("Binary S picture", temp);
}

void deterCallback_V(int, void*){
    Mat temp = hsv_split[2].clone();
    threshold(hsv_split[2], temp, V_max, MAX_SIZE, THRESH_TOZERO_INV);
    threshold(temp, temp, V_min, MAX_SIZE, THRESH_TOZERO);
    threshold(temp, temp, V_min, MAX_SIZE, THRESH_BINARY);
    imshow("Binary V picture", temp);
}

void deterCallback_v(int, void*){
    Mat temp = hsv_split[2].clone();
    threshold(hsv_split[2], temp, V_max, MAX_SIZE, THRESH_TOZERO_INV);
    threshold(temp, temp, V_min, MAX_SIZE, THRESH_TOZERO);
    threshold(temp, temp, V_min, MAX_SIZE, THRESH_BINARY);
    imshow("Binary V picture", temp);
}

void deter_thresh(){
    namedWindow("source picture", CV_WINDOW_AUTOSIZE);
    imshow("source picture", src);
    namedWindow("Binary H picture", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("H MIN:", "Binary H picture", &H_min, MAX_H_SIZE, deterCallback);
    cv::createTrackbar("H MAX:", "Binary H picture", &H_max, MAX_H_SIZE, deterCallback_1);
    deterCallback(0, 0);
    deterCallback_1(0, 0);
    cv::createTrackbar("S MIN:", "Binary S picture", &S_min, MAX_SIZE, deterCallback_S);
    cv::createTrackbar("S MAX:", "Binary S picture", &S_max, MAX_SIZE, deterCallback_s);
    deterCallback_S(0, 0);
    deterCallback_s(0, 0);
    cv::createTrackbar("V MIN:", "Binary V picture", &V_min, MAX_SIZE, deterCallback_V);
    cv::createTrackbar("V MAX:", "Binary V picture", &V_max, MAX_SIZE, deterCallback_v);
    deterCallback_V(0, 0);
    deterCallback_v(0, 0);
/*    cv::createTrackbar("S MIN:", "Binary picture", &S_min, MAX_SIZE, deterCallback);
    cv::createTrackbar("S MAX:", "Binary picture", &S_max, MAX_SIZE, deterCallback);
    cv::createTrackbar("V MIN:", "Binary picture", &V_min, MAX_SIZE, deterCallback);
    cv::createTrackbar("V MAX:", "Binary picture", &V_max, MAX_SIZE, deterCallback);*/

}

void distinctCallback(string _str, DisColor _col, Mat &_src, Mat &_dst, int _h_min, int _h_max){
    Mat src_copy = src.clone();
    vector<Vec4i> hierarchy;
    vector<vector<Point>> contours;
    threshold(_src, _dst, _h_max, MAX_SIZE, THRESH_TOZERO_INV);
    threshold(_dst, _dst, _h_min, MAX_SIZE, THRESH_TOZERO);
    threshold(_dst, _dst, _h_min, MAX_SIZE, THRESH_BINARY);

    Mat structure_element = cv::getStructuringElement(MORPH_RECT, Size(31, 31), Point(-1, -1));
    cv::erode(_dst, _dst, structure_element);
    Rect rec = cv::boundingRect(_dst);
    cv::rectangle(src_copy, rec, Scalar(255, 0, 0),1, LINE_8, 0);
    if(color_hist[_col]==0){
        for (int i=rec.tl().y; i<=rec.br().y;i++) {
            for(int j=rec.tl().x; j<=rec.br().x;j++){
                color_hist[_col]++;
            }
        }
    }
    //std::cout << _str << ": " << color_hist[_col] << std::endl;
    imshow(_str +" picture", src_copy);

}

void distinct_target(DisColor _col){
    Color_Range h_range;
    Mat temp;
    temp = hsv_split[0].clone();
    string  str;
    switch (_col) {
        case rec_red:
            str = "red";
            distinctCallback(str, _col, temp, temp, h_range.red_min[0], h_range.red_max[0]);
            break;
        case rec_orange:
            str = "orange";
            distinctCallback(str, _col, temp, temp, h_range.orange_min, h_range.orange_max);
            break;
        case rec_yellow:
            str = "yellow";
            distinctCallback(str, _col, temp, temp, h_range.yellow_min, h_range.yellow_max);
            break;
        case rec_green:
            str = "green";
            distinctCallback(str, _col, temp, temp, h_range.green_min, h_range.green_max);
            break;
        case rec_cyan:
            str = "cyan";
            distinctCallback(str, _col, temp, temp, h_range.cyan_min, h_range.cyan_max);
            break;
        case rec_blue:
            str = "blue";
            distinctCallback(str, _col, temp, temp, h_range.blue_min, h_range.blue_max);
            break;
        case rec_purple:
            str = "purple";
            distinctCallback(str, _col, temp, temp, h_range.purple_min, h_range.purple_max);
            break;
    }


}

void draw_hist(){
    Mat hist = Mat::zeros(Size(630, 800), CV_8UC3);
    int sum = 0;
    Rect r_[7];
    for (int i=0; i<7; i++) {
        sum+=color_hist[i];
        data.data[i] = float(color_hist[i]);
    }
    for (int i=0; i<7; i++) {
        percentage[i] = int(color_hist[i]*800/sum);
        Rect r(i*90, 800-percentage[i], 90, percentage[i]);
        r_[i] = r;
    }
    cv::rectangle(hist, r_[0], Scalar(0, 0, 255), -1, LINE_8, 0);
    cv::rectangle(hist, r_[1], Scalar(0, 165, 255), -1, LINE_8, 0);
    cv::rectangle(hist, r_[2], Scalar(0, 255, 255), -1, LINE_8, 0);
    cv::rectangle(hist, r_[3], Scalar(0, 255, 0), -1, LINE_8, 0);
    cv::rectangle(hist, r_[4], Scalar(255, 255, 0), -1, LINE_8, 0);
    cv::rectangle(hist, r_[5], Scalar(255, 0, 0), -1, LINE_8, 0);
    cv::rectangle(hist, r_[6], Scalar(255, 0, 255), -1, LINE_8, 0);

    imshow("hist", hist);
}


int main(int argc, char** argv){
#if IF_USE_CAMERA
    VideoCapture cap;
    cap.open(0);
#endif
    ROS_WARN("*****START*****");
    ros::init(argc, argv, "Color"); //初始化ROS节点
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/judge", 5);
    data.data.resize(7);
#if IF_USE_CAMERA
    if(!cap.isOpened()){
        ROS_INFO("Error!, Please check the camera port!");
        return 0;
    }
#endif
    while(ros::ok()){
#if IF_USE_CAMERA
        cap.read(src);
#else
        string img = "/home/demphi/ros/dip_ws/src/exp4_pkg/include/1.png";
        src = imread(img);
        if(src.empty()){
            ROS_INFO("Frame is empty!range Please check!");
            break;
        }
#endif
        color_hist = {0, 0, 0, 0, 0, 0, 0};
        cv::resize(src, src, cv::Size(0, 0), 0.6, 0.6);
        //GaussianBlur(src, src, Size(3, 3), 1.5);
        imshow("source picture", src);
        cvtColor(src, grey, COLOR_BGR2GRAY);
        cv::threshold(grey, thresh, 127, 255, THRESH_BINARY);
        RGB2HSV();
        deter_thresh();
/*        DisColor cur_color;
        cur_color = rec_red;
        distinct_target(cur_color);
        cur_color = rec_orange;
        distinct_target(cur_color);
        cur_color = rec_yellow;
        distinct_target(cur_color);
        cur_color = rec_green;
        distinct_target(cur_color);
        cur_color = rec_cyan;
        distinct_target(cur_color);
        cur_color = rec_blue;
        distinct_target(cur_color);
        cur_color = rec_purple;
        distinct_target(cur_color);
        draw_hist();
        pub.publish(data);*/
        waitKey(50);
    }
    return 0;



}