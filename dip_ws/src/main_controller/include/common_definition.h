//
// Created by demphi on 2021/11/12.
//

#ifndef MAIN_CONTROLLER_COMMON_DEFINITION_H
#define MAIN_CONTROLLER_COMMON_DEFINITION_H


#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

namespace dip{

    /*! Camera internal parameters */
    Eigen::Matrix3d intrinsic_matrix;

    /*! Include whole information that will be used during the process */
    struct StateInfo{
        /*! position in world frame*/
        geometry_msgs::Pose2D r;

        /*! velocity in world frame*/
        geometry_msgs::Twist v;

        /*! velocity model with a limit in robot frame */
        geometry_msgs::Twist v_limit;
    };

    struct ConeInfo{
        /*! class (red black)->(0 1)*/
        int color;

        /*! pixel coordinates of the center point of the cones */
        int pixel_x;
        int pixel_y;

        /*! distance */
        double distance;

        /*! position in robot frame*/
        geometry_msgs::Pose2D r_camera;

        /*! position in world frame*/
        geometry_msgs::Pose2D r_world;

    };


    /*! some common definition */
    const double kRad2Deg = atan(1.0)/45;

}


#endif 
