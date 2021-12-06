//
// Created by demphi on 2021/11/12.
//

#ifndef MAIN_CONTROLLER_STATE_UPDATE_H
#define MAIN_CONTROLLER_STATE_UPDATE_H

#include "eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "common_definition.h"
#include "kinematic_model.h"
#include "opencv2/opencv.hpp"

namespace dip{

    class StateUpdate{
    public:
        /*! flag to control target update*/
        int flag = 0;
        int count_ = 0;
        ros::NodeHandle nh_;
        geometry_msgs::Twist output_vel;
        geometry_msgs::Pose2D local_target_pose_, vision_position_;

        /*! robot and cones internal information */
        StateInfo state_info;
        ConeInfo cones_info[4];
        ConeInfo closest_red_cone_, closest_black_cone_;

        /*! msg for visualizing */
        geometry_msgs::Pose2D cur_state_;
        std_msgs::Float64MultiArray cones_info_;
        nav_msgs::Path path_info_;

        /*! other class*/
        dip::KinematicModel kinematic_model;
        dip::tools::MathTools my_tools;
        dip::Planning Bezier_path;

        /*! Can not ignore the time bias*/
        void T265Callback(const nav_msgs::OdometryConstPtr &msg);
        void CombinedSensorPositionCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
        void ConesInfoCallback(const std_msgs::Int32MultiArrayConstPtr &msg);


        StateUpdate(ros::NodeHandle &nh);
        ~StateUpdate();
        void set_timer();
        void loop(const ros::TimerEvent &);


    private:
        /*! param for bezier */
        int n_ = 0;

        /*! set the loop time*/
        ros::Timer update_timer_;

        /*! Publisher */
        ros::Publisher vel_pub_, state_pub_, cones_pub_, path_pub_;

        /*! Subscriber*/
        ros::Subscriber t265_sub_, ekf_sub_, cones_sub_;

    };

    StateUpdate::StateUpdate(ros::NodeHandle &nh) {
        this->nh_ = nh;
        this->cones_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>("/cones", 1, this);
        this->state_pub_ = this->nh_.advertise<geometry_msgs::Pose2D>("/state", 1, this);
        this->vel_pub_ = this->nh_.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5, this);
        this->path_pub_ = this->nh_.advertise<nav_msgs::Path>("/visualize/path", 1, this);
        this->t265_sub_ = this->nh_.subscribe("/camera/odom/sample", 1, &StateUpdate::T265Callback, this);
        this->ekf_sub_ = this->nh_.subscribe("/odom_combined", 1, &StateUpdate::CombinedSensorPositionCallback, this);
        this->cones_sub_ = this->nh_.subscribe("/chatter", 1, &StateUpdate::ConesInfoCallback, this);

        /*! Init some parameters*/
        this->n_ = 0;
        this->cones_info_.data.resize(6);
        this->path_info_.header.frame_id = "world";
        this->path_info_.header.stamp = ros::Time::now();
    }

    StateUpdate::~StateUpdate() {

    }

    void StateUpdate::set_timer() {
        this->update_timer_ = this->nh_.createTimer(ros::Duration(0.1), &StateUpdate::loop, this);
        return;
    }

    void StateUpdate::loop(const ros::TimerEvent &) {
        if(this->n_>=100){this->n_ = 100;}

        /*! get started to a random target */
        if(this->flag == 0 && this->n_ == 0){
            this->count_++;
            if(this->count_ > 100){
                this->local_target_pose_ = my_tools.GetTargetPosition(this->closest_red_cone_, this->closest_black_cone_);
                std::cout << "red cone" << this->closest_red_cone_.r_world << std::endl;
                std::cout << "black cone" << this->closest_black_cone_.r_world << std::endl;
                std::cout << "local target pose" << this->local_target_pose_ << std::endl;
                this->flag = 1;
                this->count_=0;
            }
            /*this->local_target_pose_.x = 1.0;
            this->local_target_pose_.y = 0.3;
            this->local_target_pose_.theta = M_PI_4;*/
            return;
        }
        if(this->n_ == 100 && this->flag == 1){
            if(abs(this->cur_state_.x-this->local_target_pose_.x)<0.1
            && abs(this->cur_state_.y-this->local_target_pose_.y)<0.1){
                this->count_++;
                if(this->count_>=10){
                    /*! update end point of bezier curve*/
                    this->n_=0;
                    this->local_target_pose_.x = this->local_target_pose_.x+0.5*cos(this->local_target_pose_.theta);
                    this->local_target_pose_.y = this->local_target_pose_.y+0.5*sin(this->local_target_pose_.theta);
                    this->local_target_pose_.theta = this->local_target_pose_.theta;
                    this->flag = 2;
                    std::cout << "target pose " << this->local_target_pose_ << std::endl;
                }
            }
        }

        geometry_msgs::Pose2D _temp_target_pose;
        Eigen::Vector2d _target_velocity, _command;

        /*! update bezier path for visualizing */
        if(this->n_==0){
            Bezier_path.GetBezierPoints(this->state_info.r, this->local_target_pose_);
        }
        geometry_msgs::PoseStamped _path_point;
        geometry_msgs::Quaternion _point_orientation;
        _point_orientation = tf::createQuaternionMsgFromYaw(0.0);
        _path_point.pose.position.z = 0.0;
        _path_point.pose.orientation.x = _point_orientation.x;
        _path_point.pose.orientation.y = _point_orientation.y;
        _path_point.pose.orientation.z = _point_orientation.z;
        _path_point.pose.orientation.w = _point_orientation.w;

        _path_point.header.stamp = ros::Time::now();
        _path_point.header.frame_id = "world";
        _path_point.pose.position.x = Bezier_path.CalculateBezierPoints(this->n_).x();
        _path_point.pose.position.y = Bezier_path.CalculateBezierPoints(this->n_).y();
        this->path_info_.poses.push_back(_path_point);
        this->n_++;

        /*! Update command for further PID control */
        _temp_target_pose.x = _path_point.pose.position.x;
        _temp_target_pose.y = _path_point.pose.position.y;
        _target_velocity.x() = Bezier_path.CalculateBezierVelocity(this->n_).x();
        _target_velocity.y() = Bezier_path.CalculateBezierVelocity(this->n_).y();

        _command = kinematic_model.CalculateWithPID(_temp_target_pose, _target_velocity, this->state_info);
        this->output_vel.linear.x = _command.x();
        this->output_vel.angular.z = _command.y();
        if( this->flag ==2 && abs(this->cur_state_.x-this->local_target_pose_.x)<0.1 && abs(this->cur_state_.y-this->local_target_pose_.y)<0.1){
            this->output_vel.linear.x = 0;
            this->output_vel.angular.z = 0;
        }

        this->vel_pub_.publish(this->output_vel);

        /*! Publish info for visualizing */
        this->state_pub_.publish(this->cur_state_);
        this->cones_pub_.publish(this->cones_info_);
        this->path_pub_.publish(this->path_info_);

    }

    /*! ------------ Callback Function ------------- */

    void StateUpdate::T265Callback(const nav_msgs::OdometryConstPtr &msg) {
        this->state_info.r.x = msg->pose.pose.position.x;
        this->state_info.r.y = msg->pose.pose.position.y;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        this->state_info.r.theta = yaw;

        /*! update info for rviz*/
        this->cur_state_.x = this->state_info.r.x;
        this->cur_state_.y = this->state_info.r.y;
        this->cur_state_.theta = yaw;
        this->state_info.v.linear.x = msg->twist.twist.linear.x;
        this->state_info.v.linear.y = msg->twist.twist.linear.y;
    }

    void StateUpdate::CombinedSensorPositionCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
        /*this->state_info.r.x = msg.pose.pose.position.x;
        this->state_info.r.y = msg.pose.pose.position.y;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        this->state_info.r.theta = yaw;

        *//*! update info for rviz*//*
        this->cur_state_.x = this->state_info.r.x;
        this->cur_state_.y = this->state_info.r.y;
        this->cur_state_.theta = yaw;*/
    }

    void StateUpdate::ConesInfoCallback(const std_msgs::Int32MultiArrayConstPtr &msg) {
        for (int i = 0; i < 4; ++i) {
            this->cones_info[i].color = msg->data[4*i];
            this->cones_info[i].pixel_x = msg->data[4*i+1];
            this->cones_info[i].pixel_y = msg->data[4*i+2];
            this->cones_info[i].distance = double(msg->data[4*i+3])/1000.0;
            if(this->cones_info[i].pixel_x != 0 && this->cones_info[i].pixel_y != 0 && this->cones_info[i].distance != 0){
                this->cones_info[i].r_camera = my_tools.Modify_camera(this->cones_info[i].color,
                                                                      this->cones_info[i].pixel_x,
                                                                      this->cones_info[i].pixel_y,
                                                                      this->cones_info[i].distance);
                this->cones_info[i].r_world = my_tools.Modify_world(this->cones_info[i], this->state_info);
            }
        }

        /*
        std::cout << " cone info in camera frame = "<<this->cones_info[0].r_camera<< std::endl;
        std::cout << " cone info in world frame = "<<this->cones_info[0].r_world<< std::endl;
        */

        /*! find the closest red cone, black cone and pair them*/
        int _k_red = -1, _k_black = -1;
        for (int i = 0; i < 4; i++) {
            if(this->cones_info[i].color == 0 && _k_red == -1 && this->cones_info[i].distance>0.3){
                _k_red = i;
            }
            if(_k_red != -1 && this->cones_info[i].color == 0 && this->cones_info[i].distance > 0.3 && (this->cones_info[i].distance < this->cones_info[_k_red].distance)){
                _k_red = i;
            }
        }
        for (int i = 0; i < 4; i++) {
            if(this->cones_info[i].color == 1 && _k_black == -1 && this->cones_info[i].distance>0.3){
                _k_black = i;
            }
            if(_k_black != -1 && this->cones_info[i].color == 1 && this->cones_info[i].distance > 0.3 && (this->cones_info[i].distance < this->cones_info[_k_black].distance)){
                _k_black = i;
            }
        }
        this->closest_red_cone_ = this->cones_info[_k_red];
        this->closest_black_cone_ = this->cones_info[_k_black];


        /*! update info for rviz*/
        this->cones_info_.data[0] = this->cones_info[_k_red].r_world.theta;
        this->cones_info_.data[1] = this->cones_info[_k_red].r_world.x;
        this->cones_info_.data[2] = this->cones_info[_k_red].r_world.y;

        this->cones_info_.data[3] = this->cones_info[_k_black].r_world.theta;
        this->cones_info_.data[4] = this->cones_info[_k_black].r_world.x;
        this->cones_info_.data[5] = this->cones_info[_k_black].r_world.y;
    }

}

#endif 
