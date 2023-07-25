/* This is the Main Handler script for spike action pck
    Its purpuse is the decision making of top level
    */
#include <iostream>
#include <string.h>
#include <Robot.h>
#include <CommunicationHandler.h>
#include <Controller.h>
#include <DataHandler.h>
#include <chrono>
#include <ctime> 

using namespace ros;

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "main_handler");
    ros::NodeHandle nh;

    ros::Rate loop_rate(500);

    // Constructor Robot, CommunicationHandler, Controller, DataHandler
    RCD::DataHandler* data_handler = new RCD::DataHandler();
    RCD::Robot* robot = new RCD::Robot();
    RCD::CommunicationHandler* cmh = new RCD::CommunicationHandler(robot, &nh);
    RCD::Controller* ctrl = new RCD::Controller(robot, cmh, data_handler);

    // Info for user
    if (!nh.getParam(robot->robot_name_ + "/robot_name", robot->robot_name_)){
        ROS_ERROR("No robot name given in namespace: '%s')", nh.getNamespace().c_str());
    }
    else
    {
        ROS_INFO("START CONROL for: '%s'", robot->robot_name_.c_str());
    }

    // Async Ros
    ros::AsyncSpinner spinner(1); // 0 means as many threads my machine gives EXTRA gibve callback queue
    spinner.start();

    // Initialize Communication Handler either simulated or real experiment
    cmh->initCommunicationHandler(); 

    // Open csv file    
    cmh->TIPS ? data_handler->openOnceTips() : data_handler->openOnce();
    
    sleep(2); // sleep for 2 second

    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Initialize Controller
    ctrl->initControl();

    sleep(1); // sleep for 1 second

    ROS_INFO("StandUp(): starts");
    // Send first cmds
    ctrl->standUp();
    ROS_INFO("StandUp(): ends");

    if(cmh->SLIP_DETECTION)
    {
        while(!cmh->ALLIMUOK() )
        {
            ROS_INFO("Wait for IMUs");
        }
    }
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // std::cout<<robot->p_c<<std::endl;
    // std::cout<<robot->p_c0<<std::endl;
    // std::cout<<robot->R_c<<std::endl;

    ROS_INFO("Control loop(): will start in 5sec");

    sleep(5); // sleep for 2 seconds

    ROS_INFO("Control loop(): starts");
    ctrl->loop();
    ROS_INFO("Control loop(): ends");

    // Close csv file    
    data_handler->closeOnce();

    ros::waitForShutdown();

    // De-Constructors
    delete cmh;
    delete ctrl;
    delete robot;

    return 0;
}
