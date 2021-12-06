//
// Created by demphi on 2021/11/11.
//

#include "stdlib.h"
#include "eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "kinematic_model.h"
#include "math_tools.h"

dip::KinematicModel kinematic_model;
geometry_msgs::Twist twist;
ros::Publisher cmd_pub_;

/*! Init some parameter*/
void ParamInit(){
    twist.linear.x=0;
    twist.linear.y=0;
    twist.linear.z=0;
    twist.angular.x=0;
    twist.angular.y=0;
    twist.angular.z=0;
}

/*!
 * @brief Pose2D msg is different of the normal
 * @param x represents the command velocity of x
 * @param y represents the command velocity of y
 * @param theta represents current yaw angle
 * */
void CommandCallback(const geometry_msgs::Pose2D &msg){
    Eigen::Vector2d output = {0.0, 0.0};
    output = kinematic_model.Calculate(msg);
    twist.linear.x = output.x();
    twist.angular.z = output.y();
    ROS_INFO("remap is working!");
    cmd_pub_.publish(twist);
}

/*!
 * */
void OdomCallback(const nav_msgs::OdometryConstPtr &msg){
    Eigen::Quaterniond quat(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);
    /*! a0, a1, a2 represents the order of rotation; (2, 1, 0)->z轴, y轴, x轴(YPR)*/
    Eigen::Vector3d angle = quat.matrix().eulerAngles(2, 1, 0);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "Move");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub_, odom_sub_;
    ParamInit();
    odom_sub_ = nh.subscribe("/odom", 1000, &OdomCallback);
    pose_sub_ = nh.subscribe("/command_velocity", 1000, &CommandCallback);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::spin();
    return 0;

}