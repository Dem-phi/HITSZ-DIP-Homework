//
// Created by demphi on 2021/11/30.
//
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "tf/transform_datatypes.h"
#include "planning.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "Test");
    ros::NodeHandle nh;
    ros::Publisher my_pub;

    nav_msgs::Path my_path;
    my_path.header.frame_id = "world";
    my_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped path_point;
    geometry_msgs::Quaternion point_orientation;
    point_orientation = tf::createQuaternionMsgFromYaw(0.0);
    path_point.pose.position.z = 0.0;
    path_point.pose.orientation.x = point_orientation.x;
    path_point.pose.orientation.y = point_orientation.y;
    path_point.pose.orientation.z = point_orientation.z;
    path_point.pose.orientation.w = point_orientation.w;

    my_pub = nh.advertise<nav_msgs::Path>("/visualize/path", 1);
    dip::Planning MyBezier;
    geometry_msgs::Pose2D start, end;
    start.x = 0;
    start.y = 0;
    start.theta = M_PI_4;
    end.x = 2.3;
    end.y = 1.8;
    end.theta = M_PI_2;
    Eigen::Vector2d temp;
    MyBezier.GetBezierPoints(start, end);
    ros::Rate rate(10);
    int n = 0;
    while(ros::ok()){
        if(n>100){
            n=0;
        }
        temp = MyBezier.CalculateBezierPoints(n);
        path_point.header.stamp = ros::Time::now();
        path_point.header.frame_id = "world";
        path_point.pose.position.x = temp.x();
        path_point.pose.position.y = temp.y();
        my_path.poses.push_back(path_point);
        my_pub.publish(my_path);
        ROS_INFO("n = %d, ans_ = %.2f, %.2f", n, temp.x(), temp.y());
        n++;
        rate.sleep();
    }
    return 0;
}
