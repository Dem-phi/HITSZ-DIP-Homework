//
// Created by demphi on 2021/11/11.
//

#ifndef MAIN_CONTROLLER_MATH_TOOLS_H
#define MAIN_CONTROLLER_MATH_TOOLS_H

#include "eigen3/Eigen/Dense"
#include "common_definition.h"
#include "planning.h"

/*!
 * @brief
 *
 * */
namespace dip{
    namespace tools{
        class MathTools{
        public:

            Planning planning;

            /*!
             * @brief Get the real position of cone in camera frame
             * @param return include [x, y, theta] represents [x, y, class] in camera frame
             * */
            geometry_msgs::Pose2D Modify_camera(int color, int pixel_x, int pixel_y, double distance);

            /*!
             * @brief Get the real position of cone in world frame
             * @param return include [x, y, theta] represents [x, y, class] in world frame
             * */
            geometry_msgs::Pose2D Modify_world(dip::ConeInfo cone, dip::StateInfo cur_state);

            /*!
             * @brief Demo of planning
             * @param red_cone always at left
             * @param black_cone always at right
             * */
            geometry_msgs::Pose2D GetTargetPosition(dip::ConeInfo red_cone, dip::ConeInfo black_cone);

            /*!
             * @brief Demo of calculating velocity
             * @param
             * */
            geometry_msgs::Pose2D CalculateVelocity(geometry_msgs::Pose2D target_goal, dip::StateInfo cur_state);

        };

        geometry_msgs::Pose2D MathTools::Modify_camera(int color, int pixel_x, int pixel_y, double distance) {
            Eigen::Matrix3d _intrinsic_matrix;
            _intrinsic_matrix << 696.565, 0.0, 657.715,
                                 0.0, 696.565, 362.6485,
                                 0.0, 0.0, 1.0;
            //_intrinsic_matrix << 337.8913879394531, 0.0, 336.7178955078125, 0.0, 337.8913879394531, 192.87644958496094, 0.0, 0.0, 1.0;
            geometry_msgs::Pose2D _temp;
            Eigen::Vector3d _pixel, _real;
            _pixel.x() = pixel_x;
            _pixel.y() = pixel_y;
            _pixel.z() = 1.0;
            _real = _intrinsic_matrix.inverse()*distance*_pixel;
            _temp.x = distance;
            _temp.y = -_real.x();
            _temp.theta = color;
            return _temp;
        }

        geometry_msgs::Pose2D MathTools::Modify_world(dip::ConeInfo cone, dip::StateInfo cur_state) {
            geometry_msgs::Pose2D _temp;
            _temp.x = cone.r_camera.x*cos(cur_state.r.theta)-cone.r_camera.y*sin(cur_state.r.theta)+cur_state.r.x;
            _temp.y = cone.r_camera.x*sin(cur_state.r.theta)+cone.r_camera.y*cos(cur_state.r.theta)+cur_state.r.y;
            /*! update class to world frame for publishing to rviz*/
            _temp.theta = cone.color;
            return _temp;
        }


        geometry_msgs::Pose2D MathTools::GetTargetPosition(dip::ConeInfo red_cone, dip::ConeInfo black_cone) {
            geometry_msgs::Pose2D _target_pose;
            _target_pose.x = (red_cone.r_world.x + black_cone.r_world.x)/2;
            _target_pose.y = (red_cone.r_world.y + black_cone.r_world.y)/2;
            /*! black in the left, red in the right*/
            if(black_cone.r_world.x - red_cone.r_world.x > 0.03){
                // theta > 90
                _target_pose.theta = -atan(abs(black_cone.r_world.x - red_cone.r_world.x)/abs(black_cone.r_world.y - red_cone.r_world.y));
            }else if(black_cone.r_world.x - red_cone.r_world.x < -0.03){
                // theta < 90
                _target_pose.theta = atan(abs(black_cone.r_world.x - red_cone.r_world.x)/abs(black_cone.r_world.y - red_cone.r_world.y));
            }else{
                _target_pose.theta = 0;
            }

            return _target_pose;
        }


        geometry_msgs::Pose2D MathTools::CalculateVelocity(geometry_msgs::Pose2D target_goal, dip::StateInfo cur_state) {
            /*geometry_msgs::Pose2D _target_vel;
            float dT = 10;
            _target_vel.x = (target_goal.x-cur_state.r.x)/dT;
            _target_vel.y = (target_goal.y-cur_state.r.y)/dT;
            _target_vel.theta = target_goal.theta;
            return _target_vel;*/
            /*! Get the arc trajectory velocity*/
            geometry_msgs::Pose2D _target_vel;
            Eigen::Vector2d _arc_trajectory;
            _arc_trajectory = planning.GetArcTrajectory(cur_state, target_goal);
            _target_vel.x = _arc_trajectory.x()*cos(cur_state.r.theta);
            _target_vel.y = _arc_trajectory.x()*sin(cur_state.r.theta);
            _target_vel.theta = _arc_trajectory.y();
            return _target_vel;


        }
    }
}


#endif 
