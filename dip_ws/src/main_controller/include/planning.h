//
// Created by demphi on 2021/11/26.
//

#ifndef MAIN_CONTROLLER_PLANNING_H
#define MAIN_CONTROLLER_PLANNING_H
#include "ros/ros.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float64MultiArray.h"
#include "common_definition.h"


namespace dip{
    class Planning{
    public:
        /*! return the target orientation at the start point
         * @param (v_max, theta0)*/
        Eigen::Vector2d GetArcTrajectory(dip::StateInfo cur_state, geometry_msgs::Pose2D target_pose);

        /*! */
        void GetBezierPoints(geometry_msgs::Pose2D start_point, geometry_msgs::Pose2D end_point);
        /*!
         * @brief get target points and target velocity
         * @param n[0, this->num_]
         * */
        Eigen::Vector2d CalculateBezierPoints(double n);

        Eigen::Vector2d CalculateBezierVelocity(double n);

        Planning();
        ~Planning();

    private:
        double v_max_ = 0.1;

        /*! */
        double offset_ = 3;
        double distance_;
        double num_ = 100;
        Eigen::Vector2d start_point_, end_point_;
        Eigen::Vector2d assistant_point_[2];

    };

    Planning::Planning() {

    }

    Planning::~Planning() {

    }

    Eigen::Vector2d Planning::GetArcTrajectory(dip::StateInfo cur_state, geometry_msgs::Pose2D target_pose) {
        Eigen::Vector2d _ans;
        _ans.x() = this->v_max_;
        _ans.y() = 2*atan((target_pose.y-cur_state.r.y)/(target_pose.x-cur_state.r.x))-target_pose.theta;
        return _ans;
    }

    void Planning::GetBezierPoints(geometry_msgs::Pose2D start_point, geometry_msgs::Pose2D end_point) {
        this->distance_ = sqrt(pow((end_point.x-start_point.x),2)+pow((end_point.y-start_point.y), 2))/this->offset_;
        this->assistant_point_[0].x() = start_point.x + this->distance_*cos(start_point.theta);
        this->assistant_point_[0].y() = start_point.y + this->distance_*sin(start_point.theta);
        this->assistant_point_[1].x() = end_point.x - this->distance_*cos(end_point.theta);
        this->assistant_point_[1].y() = end_point.y - this->distance_*sin(end_point.theta);

        this->start_point_.x() = start_point.x;
        this->start_point_.y() = start_point.y;
        this->end_point_.x() = end_point.x;
        this->end_point_.y() = end_point.y;
    }

    Eigen::Vector2d Planning::CalculateBezierPoints(double n) {
        double t;
        if(n > this->num_){
            t = 1;
        }else if(n < 0){
            t = 0;
        }else{
            t = n/this->num_;
        }

        Eigen::Vector2d ans_;
        ans_ =  this->start_point_*pow((1-t),3)
                + 3*this->assistant_point_[0]*t*pow((1-t),2)
                + 3*this->assistant_point_[1]*pow(t, 2)*(1-t)
                + this->end_point_*pow(t, 3);
        //ROS_INFO("t = %.2f, ans_ = %.2f, %.2f", t, ans_.x(), ans_.y());
        return ans_;
    }

    Eigen::Vector2d Planning::CalculateBezierVelocity(double n) {
        double t;
        if(n > this->num_){
            t = 1;
        }else if(n < 0){
            t = 0;
        }else{
            t = n/this->num_;
        }
        Eigen::Vector2d _ans;
        _ans = (-3)*this->start_point_* pow((1-t), 2)
                +3*this->assistant_point_[0]* pow((1-t),2) - 6*this->assistant_point_[0]*t*(1-t)
                +6*this->assistant_point_[1]*t*(1-t) - 3*this->assistant_point_[1]* pow(t, 2)
                +3*this->end_point_* pow(t, 2);

        return _ans;
    }

}



#endif 
