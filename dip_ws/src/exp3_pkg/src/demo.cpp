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
#include <algorithm>

#define LINEAR_X 0
#define IF_USE_CAMERA 1

using namespace cv;
using namespace std;


Mat src, grey, thresh;
Mat dst_canny, dst_line, dst_circle;
int hough_value = 120;

/*!
 * @param input
 * @param output
 */

void EdgeDetector(){
    cv::Canny(src, dst_canny, 50, 200);
    cv::namedWindow("Canny", CV_WINDOW_NORMAL);
    imshow("Canny", dst_canny);
}

/*!
 * @param rho   the minimum resolution of distance (pixels)
 * @param theta the minimum resolution of angle
 * @param threshold
 */
void Hough_Line(Mat output, float rho, float theta, int threshold) {
    vector<Vec2f> lines;
    cv::Canny(grey, dst_canny, 50, 200, 3);
    cv::HoughLines(dst_canny, lines, rho, theta, threshold, 0, 0);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        //提取出距离和角度
        float rho = lines[i][0], theta = lines[i][1];
        //定义两个点，两点确定一条直线
        //计算得到的两点的坐标为（ρcosθ-1000sinθ，ρsinθ+1000cosθ），（ρcosθ+1000sinθ，ρsinθ-1000cosθ）
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        //在原图上画宽带为2的红线
        line( src, pt1, pt2, Scalar(0,0,255),2);
    }
    namedWindow( "lines", CV_WINDOW_AUTOSIZE );
    imshow( "lines", src );
}

/*!
 * @brief as a callback
 */
void Hough_Circle(){
    vector<Vec3f>circles;
    //minDist 和 param2 数值的设定是关键
    HoughCircles(grey, circles, HOUGH_GRADIENT, 1, 10, 130);
    Mat show = src.clone();
    for (int i = 0; i < circles.size(); i++) {
        circle(show, Point(circles[i][0], circles[i][1]), circles[i][2], Scalar(0, 0, 255), 2);
    }
    imshow("Hough Circles", show);
}

int main(int argc, char** argv){
#if IF_USE_CAMERA
    VideoCapture cap;
    cap.open(0);
#endif
    ROS_WARN("*****START*****");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);
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
        string img = "/home/demphi/ros/dip_ws/src/exp3_pkg/include/5.jpeg";
        src = imread(img);
        if(src.empty()){
            ROS_INFO("Frame is empty! Please check!");
            break;
        }
#endif
        imshow("source picture", src);
        cvtColor(src, grey, COLOR_BGR2GRAY);
        cv::threshold(grey, thresh, 127, 255, THRESH_BINARY);
        //EdgeDetector();
        Hough_Line(dst_line, 10, CV_PI/180, 1000);
        //Hough_Circle();
        waitKey(10);
    }
    return 0;



}