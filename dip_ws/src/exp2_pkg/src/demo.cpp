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

#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"

#define LINEAR_X 0

using namespace cv;
using namespace std;

Mat src, grey, thresh;
Mat dst_gas;
Mat dst_dilate, dst_erode;
// for dilate
int element_size = 3;
int max_size = 21;


void Gaussian(Mat input, Mat output, double sigma, int size){
    Mat core(size, size, CV_64FC1);
    cv::Size c_size(size, size);
    // create gaussian core
/*    core(0, 0) = 1/(2*M_PI*sigma*sigma)*exp(-1/(sigma*sigma));
    core(0, 2) = 1/(2*M_PI*sigma*sigma)*exp(-1/(sigma*sigma));
    core(2, 0) = 1/(2*M_PI*sigma*sigma)*exp(-1/(sigma*sigma));
    core(2, 2) = 1/(2*M_PI*sigma*sigma)*exp(-1/(sigma*sigma));
    core(0, 1) = 1/(2*M_PI*sigma*sigma)*exp(-1/(2*sigma*sigma));
    core(1, 0) = 1/(2*M_PI*sigma*sigma)*exp(-1/(2*sigma*sigma));
    core(1, 2) = 1/(2*M_PI*sigma*sigma)*exp(-1/(2*sigma*sigma));
    core(2, 1) = 1/(2*M_PI*sigma*sigma)*exp(-1/(2*sigma*sigma));
    core(1, 1) = 1/(2*M_PI*sigma*sigma);*/
    /*int center = size/2;
    double sum = 0;
    for(int i=0; i<size; i++){
        for(int j=0; j<size; j++){
            core.at<double>(i, j) = (1/(2*M_PI*sigma*sigma))*exp(-((i-center)*(i-center)+(j-center)*(j-center))/(2*sigma*sigma));
            sum += core.at<double>(i, j);
        }
    }
    for(int i=0; i<size; i++){
        for(int j=0; j<size; j++){
            core.at<double>(i,j) = core.at<double>(i,j)/sum;
        }
    }
    //cout << core << endl;
    Mat input_copy = input.clone();
    Mat output_copy = input_copy.clone();

    for(int x=(size-1)/2; x<input_copy.cols-(size-1)/2; x++) {
        for (int y=(size-1)/2; y<input_copy.rows-(size-1)/2; y++){
            double sum = 0;
            for (int s = -((size-1)/2); s<=(size-1)/2; s++) {
                for (int t = -((size-1)/2); t<=(size-1)/2; t++) {
                    sum += core.at<double>((size-1)/2+s, (size-1)/2+t)*input_copy.at<uchar>(x-s, y-t);
                }
            }
            output_copy.at<uchar>(x, y) = int(sum);
        }
    }
    output = output_copy.clone();
*/
    Mat o_output;
    cv::GaussianBlur(input, o_output, c_size, sigma);

    imshow("input", input);
    //imshow("output", output);
    imshow("output_o", o_output);
}

void dilate_callback(int, void*){
    int s = element_size*2+1;
    // create the element structure
    Mat structure_element = cv::getStructuringElement(MORPH_RECT, Size(s, s), Point(-1, -1));
    cv::dilate(thresh, dst_dilate, structure_element, Point(-1, -1), 1);
    imshow("After dilate", dst_dilate);
}

void Dilate(){
    namedWindow("Binary picture", CV_WINDOW_AUTOSIZE);
    imshow("Binary picture", thresh);
    namedWindow("After dilate", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("Element size:", "After dilate", &element_size, max_size, dilate_callback);
    dilate_callback(element_size, 0);
}

void erode_callback(int, void*){
    int s = element_size*2+1;
    // create the element structure
    Mat structure_element = cv::getStructuringElement(MORPH_RECT, Size(s, s), Point(-1, -1));
    cv::erode(thresh, dst_erode, structure_element);
    imshow("After erode", dst_erode);
}

void Erode(){
    namedWindow("Binary picture", CV_WINDOW_AUTOSIZE);
    imshow("Binary picture", thresh);
    namedWindow("After erode", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("Element size:", "After erode", &element_size, max_size, erode_callback);
    erode_callback(element_size, 0);
}

int main(int argc, char** argv){
    VideoCapture cap;
    cap.open(2);

    ROS_WARN("*****START*****");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);
    if(!cap.isOpened()){
        ROS_INFO("Error!, Please check the camera port!");
        return 0;
    }
    /*waitKey(1000);
    Mat frame;
    int cur_frame = 0;
    int frameWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frameHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);*/
    /*String img = "/home/demphi/ros/dip_ws/src/exp2_pkg/include/img.png";
    src = imread(img);
    if(src.empty()){
        std::cout << "Error! Can not load the picture!" << std::endl;
    }*/

    //Dilate();
    //Erode();

    while(ros::ok()){
        cap.read(src);
        if(src.empty()){
            ROS_INFO("Frame is empty! Please check!");
            break;
        }
        cap.read(src);
        imshow("source picture", src);
        cvtColor(src, grey, COLOR_BGR2GRAY);
        cv::threshold(grey, thresh, 127, 255, THRESH_BINARY);
        dst_gas = src.clone();

        //Gaussian(grey, dst_gas, 10, 11);
        //Dilate();
        Erode();
        waitKey(10);


    }
    return 0;



}