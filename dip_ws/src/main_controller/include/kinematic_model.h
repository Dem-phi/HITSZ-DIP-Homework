//
// Created by demphi on 2021/11/11.
//

#ifndef EXP4_PKG_KINEMATIC_MODEL_H
#define EXP4_PKG_KINEMATIC_MODEL_H

#include "geometry_msgs/Pose2D.h"
#include "eigen3/Eigen/Dense"
#include "common_definition.h"
#include "math_tools.h"
#include "planning.h"

namespace dip{
    /*! Mapping frame is camera, Drive frame is car itself*/
    class KinematicModel{
    public:
        /*! we will use the camera coordinate as the base frame */
        double kBiasToCar = 0.1;
        double kRad2Deg = atan(1.0)/45;

        double kp = 0.1;
        double kd = 0.1;

        double kp_w = 0.5;
        double kd_w = 0.01;


        dip::tools::MathTools math_tools;

        /*! [ linear velocity, angular velocity ]*/
        Eigen::Vector2d input_velocity_ = {0.0, 0.0};
        Eigen::Vector2d output_ = {0.0, 0.0};
        Eigen::Matrix2d transform_matrix_ ;

        /*!
         * @brief Transform the command velocity (Vx, Vy) in world frame to robot model velocity
         * @param command_state_ (x, y, theta) represents (Vx in world frame, Vy in world frame, current angle in world frame)
         * */
        Eigen::Vector2d CalculateWithPID(geometry_msgs::Pose2D target_pose, Eigen::Vector2d target_vel, dip::StateInfo cur_state);

        /*!
         * @brief Velocity controller
         * @param
         * */
        geometry_msgs::Pose2D PIDController(geometry_msgs::Pose2D target_pose, Eigen::Vector2d target_vel, dip::StateInfo cur_state);

        /*!
         * @brief Correct the angle
         * @param
         * */
        Eigen::Vector2d CorrectOrientation(geometry_msgs::Pose2D target_pose, dip::StateInfo cur_state);

        KinematicModel();
        ~KinematicModel();
    };

    KinematicModel::KinematicModel() {
        this->transform_matrix_.setIdentity();
    }

    KinematicModel::~KinematicModel() {
    }

    Eigen::Vector2d KinematicModel::CalculateWithPID(geometry_msgs::Pose2D target_pose, Eigen::Vector2d target_vel, dip::StateInfo cur_state) {
        this->transform_matrix_ (0,0) = cos(cur_state.r.theta);
        this->transform_matrix_(0,1) = sin(cur_state.r.theta);
        this->transform_matrix_(1,0) = -sin(cur_state.r.theta)/this->kBiasToCar;
        this->transform_matrix_(1,1) = cos(cur_state.r.theta)/this->kBiasToCar;

        this->input_velocity_.x() = target_vel.x();
        this->input_velocity_.y() = target_vel.y();
        this->output_ = this->transform_matrix_*this->input_velocity_;

        double _distance = cos(cur_state.r.theta)*sqrt(pow((target_pose.x-cur_state.r.x),2)+pow((target_pose.y-cur_state.r.y),2));

        double _vel_yaw = atan(target_vel.y()/target_vel.x());

        this->output_.x() = this->kd*this->output_.x()+this->kp*_distance*cos(cur_state.r.theta);
        this->output_.y() = this->kd_w*this->output_.y()+this->kp_w*(_vel_yaw-cur_state.r.theta);
        if(abs(cur_state.r.x - target_pose.x) < 0.05 && abs(cur_state.r.y - target_pose.y) < 0.05){
            this->output_.x() = 0;
            this->output_.y() = 0;
        }
        //std::cout << this->output_ << std::endl;

        return this->output_;
    }


    geometry_msgs::Pose2D KinematicModel::PIDController(geometry_msgs::Pose2D target_pose, Eigen::Vector2d target_vel, dip::StateInfo cur_state) {
        /*! 轨迹跟踪 */
        geometry_msgs::Pose2D _command_vel;
        _command_vel.theta = cur_state.r.theta;

        _command_vel.x = this->kd*target_vel.x()+this->kp*(target_pose.x-cur_state.r.x);
        _command_vel.y = this->kd*target_vel.y()+this->kp*(target_pose.y-cur_state.r.y);

        return _command_vel;

        /*! planning之后，需要速度解算 */
/*        double dT = 10;
        geometry_msgs::Pose2D target_vel;

        target_vel.theta = cur_state.r.theta;
        target_vel.x = kp*(target_pose.x-cur_state.r.x)+kd*(target_pose.x-cur_state.r.x)/dT;
        target_vel.y = kp*(target_pose.y-cur_state.r.y)+kd*(target_pose.y-cur_state.r.y)/dT;
        return target_vel;*/


    }


    Eigen::Vector2d KinematicModel::CorrectOrientation(geometry_msgs::Pose2D target_pose, dip::StateInfo cur_state) {
        double _error;
        _error = target_pose.theta-cur_state.r.theta;

        Eigen::Vector2d _command;
        _command.x() = 0.0;
        _command.y() = kp_w*_error;

        return _command;
    }

}




#endif 
