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


    // std::cout << "MAESTRO CONTROL: " << std::endl
    //           << "Make sure the robot is standing on the ground." << std::endl
    //           << "Press Enter: Next Starting Pose..." << std::endl;
    // std::cin.ignore();

    sleep(3); // sleep for 3 seconds

    // Initialize Controller
    ctrl->initControl();


    // std::cout << "MAESTRO CONTROL: " << std::endl
    //           << "Make sure the robot is standing on the ground." << std::endl
    //           << "Press Enter: Next StandUp..." << std::endl;
    // std::cin.ignore();

    sleep(2); // sleep for 3 seconds

    ROS_INFO("StandUp(): starts");
    // Send first cmds
    ctrl->standUp();
    ROS_INFO("StandUp(): ends");

    if(cmh->SLIP_DETECTION)
    {
        while(!cmh->IMU_OK )
        {
            ROS_INFO("Wait for IMUs");
        }
    }

    sleep(4); // sleep for 5 seconds

    ROS_INFO("Control loop(): starts");
    ctrl->loop();
    ROS_INFO("Control loop(): ends");

    ros::waitForShutdown();

    // De-Constructors
    delete cmh;
    delete ctrl;
    delete robot;

    return 0;
}
