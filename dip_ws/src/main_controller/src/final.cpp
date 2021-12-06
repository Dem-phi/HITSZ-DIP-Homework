//
// Created by demphi on 2021/11/12.
//

#include "state_update.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "MainController");
    ros::NodeHandle nh;

    dip::StateUpdate MyState(nh);
    MyState.set_timer();

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;

}