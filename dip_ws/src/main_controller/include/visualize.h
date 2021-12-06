//
// Created by demphi on 2021/11/26.
//

#ifndef MAIN_CONTROLLER_VISUALIZE_H
#define MAIN_CONTROLLER_VISUALIZE_H


#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float64MultiArray.h"
#include "tf/transform_datatypes.h"
#include "planning.h"

namespace visualize{
    class Visualize{
    private:
        int n_;
        double tolerance_ = 0.1;
        /*! set the loop timer*/
        ros::Timer update_timer_;

        /*! Publish to rviz */
        ros::Publisher route_pub_, cones_pub_;

        /*! Subscriber */
        ros::Subscriber state_sub_, cones_sub_;

    public:
        ros::NodeHandle nh_;

        geometry_msgs::Pose2D cur_state_;
        nav_msgs::Path route_;
        geometry_msgs::Pose2D red_cone_position_, black_cone_position_;

        visualization_msgs::Marker red_cone_, black_cone_;

        /*! update variables */
        void update(const ros::TimerEvent &);
        bool draw_cones(visualization_msgs::Marker last_position, visualization_msgs::Marker cur_position);

        void set_timer();
        Visualize(ros::NodeHandle &nh);
        ~Visualize();

        void StateCallback(const geometry_msgs::Pose2D &msg);
        void ConesCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
    };

    Visualize::Visualize(ros::NodeHandle &nh) {
        this->nh_ = nh;
        this->route_pub_ = this->nh_.advertise<nav_msgs::Path>("/visualize/route", 1, this);
        this->cones_pub_ = this->nh_.advertise<visualization_msgs::Marker>("/visualize/cones", 1, this);
        this->state_sub_ = this->nh_.subscribe("/state", 1000, &Visualize::StateCallback, this);
        this->cones_sub_ = this->nh_.subscribe("/cones", 1000, &Visualize::ConesCallback, this);

        /*! Init parameters*/
        this->route_.header.frame_id = "world";
        this->route_.header.stamp = ros::Time::now();
        this->n_ = 0;

        this->red_cone_position_.theta = 0;
        this->black_cone_position_.theta = 1;

        this->red_cone_.header.frame_id = "world";
        this->red_cone_.header.stamp = ros::Time::now();
        this->red_cone_.ns = "red cones";
        this->red_cone_.type = visualization_msgs::Marker::CYLINDER;
        this->red_cone_.action = visualization_msgs::Marker::ADD;
        this->red_cone_.pose.position.z = 0.0;
        this->red_cone_.pose.orientation.x = 0.0;
        this->red_cone_.pose.orientation.y = 0.0;
        this->red_cone_.pose.orientation.z = 0.0;
        this->red_cone_.pose.orientation.w = 0.0;
        this->red_cone_.scale.x = 1.0;
        this->red_cone_.scale.y = 0.1;
        this->red_cone_.scale.z = 0.1;
        this->red_cone_.color.a = 1.0;
        this->red_cone_.color.r = 1.0;
        this->red_cone_.color.g = 0.0;
        this->red_cone_.color.b = 0.0;

        this->black_cone_.header.frame_id = "world";
        this->black_cone_.header.stamp = ros::Time::now();
        this->black_cone_.ns = "red cones";
        this->black_cone_.type = visualization_msgs::Marker::CYLINDER;
        this->black_cone_.action = visualization_msgs::Marker::ADD;
        this->black_cone_.pose.position.z = 0.0;
        this->black_cone_.pose.orientation.x = 0.0;
        this->black_cone_.pose.orientation.y = 0.0;
        this->black_cone_.pose.orientation.z = 0.0;
        this->black_cone_.pose.orientation.w = 0.0;
        this->black_cone_.scale.x = 1.0;
        this->black_cone_.scale.y = 0.1;
        this->black_cone_.scale.z = 0.1;
        this->black_cone_.color.a = 1.0;
        this->black_cone_.color.r = 1.0;
        this->black_cone_.color.g = 1.0;
        this->black_cone_.color.b = 1.0;


    }

    Visualize::~Visualize() {

    }

    void Visualize::update(const ros::TimerEvent &) {
        geometry_msgs::PoseStamped _cur_pose_stamped;
        geometry_msgs::Quaternion _cur_orientation;
        _cur_orientation = tf::createQuaternionMsgFromYaw(this->cur_state_.theta);

        _cur_pose_stamped.header.frame_id = "world";
        _cur_pose_stamped.header.stamp = ros::Time::now();
        _cur_pose_stamped.pose.position.x = this->cur_state_.x;
        _cur_pose_stamped.pose.position.y = this->cur_state_.y;
        _cur_pose_stamped.pose.position.z = 0.0;
        _cur_pose_stamped.pose.orientation.x = _cur_orientation.x;
        _cur_pose_stamped.pose.orientation.y = _cur_orientation.y;
        _cur_pose_stamped.pose.orientation.z = _cur_orientation.z;
        _cur_pose_stamped.pose.orientation.w = _cur_orientation.w;

        this->route_.poses.push_back(_cur_pose_stamped);
        this->route_pub_.publish(this->route_);

    }

    bool Visualize::draw_cones(visualization_msgs::Marker last_position, visualization_msgs::Marker cur_position) {
        if(abs(last_position.pose.position.x - cur_position.pose.position.x) < this->tolerance_ &&
           abs(last_position.pose.position.y - cur_position.pose.position.y) < this->tolerance_){
            /*! 认为是同一个障碍物 */
            return true;
        }else{
            /*! 认为是不同的障碍物 */
            return false;
        }
    }

    void Visualize::set_timer() {
        this->update_timer_ = this->nh_.createTimer(ros::Duration(0.1), &Visualize::update, this);
        return;
    }

    /*! Callback Function */
    void Visualize::StateCallback(const geometry_msgs::Pose2D &msg) {
        this->cur_state_ = msg;
    }

    void Visualize::ConesCallback(const std_msgs::Float64MultiArrayConstPtr &msg) {
        if (msg->data[0] == this->red_cone_position_.theta){
            this->red_cone_position_.x = msg->data[1];
            this->red_cone_position_.y = msg->data[2];
            this->black_cone_position_.x = msg->data[4];
            this->black_cone_position_.y = msg->data[5];
        }else{
            this->black_cone_position_.x = msg->data[1];
            this->black_cone_position_.y = msg->data[2];
            this->red_cone_position_.x = msg->data[4];
            this->red_cone_position_.y = msg->data[5];
        }
    }



}


#endif 
