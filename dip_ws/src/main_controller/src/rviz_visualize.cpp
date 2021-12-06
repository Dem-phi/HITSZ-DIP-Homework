//
// Created by demphi on 2021/11/26.
//
#include "visualize.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "RvizVisualize");
    ros::NodeHandle nh;
    visualize::Visualize MyVisualize(nh);

    MyVisualize.set_timer();
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;

}
