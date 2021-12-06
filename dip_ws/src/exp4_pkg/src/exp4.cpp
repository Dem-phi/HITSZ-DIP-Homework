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
#include "boost/thread.hpp"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "algorithm"

using namespace cv;
using namespace std;

geometry_msgs::Twist twist;
ros::Publisher cmd_pub;

void CommandCallback(const std_msgs::Float64MultiArray& msg){
    float go = 0;
    int flag = -1;
    for (int i=0; i<7;i++) {
        if(go<msg.data[i]){
            go = msg.data[i];
            flag = i;
        }
    }
    std::cout << flag << std::endl;
    if(flag == 0){
        //red
        twist.linear.x = 0.1;
        twist.angular.z = 0;
    }
    else if(flag == 1 || flag == 2){
        //yellow
        twist.linear.x = 0.08;
        twist.angular.z = -0.25;
    }
    else if(flag == 3){
        //green
        twist.linear.x = -0.1;
        twist.angular.z = 0;
    }
    else if(flag == 5){
        //blue
        twist.linear.x = 0.08;
        twist.angular.z = 0.25;
    }
    else{
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    cmd_pub.publish(twist);
}

int main(int argc, char** argv){
    ROS_WARN("*****START*****");
    ros::init(argc, argv, "Move"); //初始化ROS节点
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Subscriber cmd_sub = nh.subscribe("/judge", 1000, &CommandCallback);
    ros::spin();
    return 0;

}

