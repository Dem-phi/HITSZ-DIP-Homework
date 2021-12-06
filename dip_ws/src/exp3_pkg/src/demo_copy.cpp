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
#define IF_USE_CAMERA 0

using namespace cv;
using namespace std;


Mat src, grey, thresh;
Mat dst_canny, dst_line, dst_circle;

/*
typedef struct CvLinePolar{
    float rho;
    float angle;
}CvLinePolar;
*/

void EdgeDetector(Mat input, Mat output){

}

/*!
 * @param rho   the minimum resolution of distance (pixels)
 * @param theta the minimum resolution of angle
 * @param threshold
 */
void Hough_Line(Mat output, float rho, float theta, int threshold) {
    cv::Canny(src, dst_canny, 50, 200, 3);
    imshow("edge", dst_canny);
    vector<Vec2f> lines;
    int total = 0;
    int step = dst_canny.step;
    int width = dst_canny.cols;
    int height = dst_canny.rows;
    int numangle = cvRound(CV_PI/theta);
    int numrho = cvRound(((width+height)*2+1)/rho);

    // allocate memory space
    cv::AutoBuffer<int> _accum, _sort_buf;
    cv::AutoBuffer<float> _tabSin, _tabCos;
    _accum.allocate((numangle+2)*(numrho+2));
    _sort_buf.allocate(numangle*numrho);
    _tabSin.allocate(numangle);
    _tabCos.allocate(numangle);

    int *accum = _accum, *sort_buf = _sort_buf;
    float *tabSin = _tabSin, *tabCos = _tabCos;
    memset(accum, 0, sizeof(accum[0])*(numangle+2)*(numrho+2));
/*    for(int i=0; i<sizeof(_accum);i++){
        accum[i] = 0;
    }*/
    // init the table of sin and cos
    float _angle_ = 0;
    for (int n = 0; n<numangle; _angle_+= theta, n++) {
        tabSin[n] = (float)(sin((double)_angle_)/rho);
        tabCos[n] = (float)(sin((double)_angle_)/rho);
    }

    //accumulate
    for (int i=0; i<height; i++) {
        for (int j=0; j<width; j++) {
            if(dst_canny.data[i*step+j] != 0){
                for(int n=0; n<numangle; n++){
                    int r = cvRound(j*tabCos[n]+i*tabSin[n]);
                    r += (numrho-1)/2;
                    accum[(n+1)*(numrho+2)+r+1]++;
                }
            }
        }
    }

    // find the local maximum for fast sort
    for (int r = 0; r < numrho; r++) {
        for (int n = 0; n < numangle; n++) {
            int base = (n+1) * (numrho+2) + r + 1;
            if( accum[base] > threshold &&
                accum[base] > accum[base-1] && accum[base] >= accum[base+1] &&
                accum[base] > accum[base-numrho-2] && accum[base] >= accum[base+numrho+2]){
                sort_buf[total++] = base;
            }
        }
    }
    std::sort(sort_buf, sort_buf+total);
    double scale = 1./(numrho+2);
    for (int i = 0; i < total; i++) {
        int idx = sort_buf[i];
        int n = cvFloor(idx*scale)-1;
        int r = idx-(n+1)*(numrho+2)-1;
        lines[i][0] = (r - (numrho - 1)*0.5f) * rho;
        lines[i][1] = n * theta;
        Point pt1, pt2;
        double a = cos(lines[i][1]), b = sin(lines[i][1]);
        double x0 = a*lines[i][0], y0 = b*lines[i][0];
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        //在原图上画宽带为2的红线
        line( src, pt1, pt2, Scalar(0,0,255),2);
    }
    imshow("find lines", src);

}
void Hough_Circle(Mat output){

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
        string img = "/home/demphi/ros/dip_ws/src/exp3_pkg/include/1.jpeg";
        src = imread(img);
#endif
        if(src.empty()){
            ROS_INFO("Frame is empty! Please check!");
            break;
        }
        imshow("source picture", src);
        cvtColor(src, grey, COLOR_BGR2GRAY);
        vector<Vec2f> lines;
        cv::threshold(grey, thresh, 127, 255, THRESH_BINARY);
        cv::Canny(src, dst_canny, 50, 200, 3);
/*        cv::HoughLines(dst_canny, lines, 2, CV_PI/180, 200, 0, 0);
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
        imshow( "lines", src );*/
        //Hough_Circle(dst_houghline);
        //Hough_Circle(dst_circle, )
        Hough_Line(dst_line, 10, CV_PI/180, 200);
        waitKey(10);
    }
    return 0;



}