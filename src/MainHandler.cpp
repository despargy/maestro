/* This is the Main Handler script for spike action pck
    Its purpuse is the decision making of top level
    */
#include <iostream>
#include <string.h>
#include <Robot.h>
#include <CommunicationHandler.h>
#include <Controller.h>
#include <chrono>
#include <ctime> 

using namespace ros;
int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "main_handler");
    ros::NodeHandle nh;

    ros::Rate loop_rate(500);

    // Constructor Robot, CommunicationHandler, Controller
    RCD::Robot* robot = new RCD::Robot();
    RCD::CommunicationHandler* cmh = new RCD::CommunicationHandler(robot, &nh);
    RCD::Controller* ctrl = new RCD::Controller(robot, cmh);

    // Info for user
    if (!nh.getParam(robot->robot_name_ + "/robot_name", robot->robot_name_)){
        ROS_ERROR("No robot name given in namespace: '%s')", nh.getNamespace().c_str());
    }
    else
    {
        ROS_INFO("START CONROL for: '%s'", robot->robot_name_.c_str());
    }

    // Async Ros
    ros::AsyncSpinner spinner(4); // 0 means as many threads my machine gives EXTRA gibve callback queue
    spinner.start();

    // Initialize Communication Handler either simulated or real experiment
    cmh->initCommunicationHandler(); 

    //Time counter START
    // ctrl->time_start = std::chrono::system_clock::now();
    sleep(3); // sleep for 3 seconds
    // ctrl->time_end = std::chrono::system_clock::now();
    // ctrl->time_elapsed =  ctrl->time_end - ctrl->time_start;
    // std::cout << "elapsed time: " << ctrl->time_elapsed.count() << "s\n";
    // ros::Duration(3).sleep(); // sleep for 5 seconds
    // Initialize Controller
    ctrl->initControl();

    // Send first cmds
    ctrl->standUp();
    sleep(5); // sleep for 3 seconds

    ctrl->loop();

    // NOT WORKING DO NOT TRY UNCOMMENTS need to smoothdown
    // ctrl->initMotorParamsHard();
    // sleep(5);

    ros::waitForShutdown();

    // De-Constructors
    delete cmh;
    delete ctrl;
    delete robot;

    return 0;
}
