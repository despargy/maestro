/* This is the Main Handler script for spike action pck
    Its purpuse is the decision making of top level
    */
#include <iostream>
#include <string.h>
#include <Robot.h>
#include <CommunicationHandler.h>
#include <Controller.h>
#include <ros/ros.h>

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

    if (!nh.getParam(robot->robot_name_ + "/robot_name", robot->robot_name_)){
        ROS_ERROR("No is_simulation given in namespace: '%s')", nh.getNamespace().c_str());
    }

    // Async Ros
    ros::AsyncSpinner spinner(4); // 0 means as many threads my machine gives EXTRA gibve callback queue
    spinner.start();

    // Initialize cmh
    cmh->initCommunicationHandler(); 
    //  subscribe to "Simulated Robot", contains pub LowCmd / sub LowState 
    cmh->initCommunicationHandlerGazebo();

    // Initialize Controller
    if( ctrl->initController()) ROS_ERROR("Fail to Inialize Controller");
    else ROS_INFO("Successfully Inialize Controller");

    ros::Duration(3).sleep(); // sleep for 5 seconds

    // Motor Params for Gazebo Control
    ctrl->initMotorParams();
    
    // Send first cmds
    ctrl->standUp();
    // ROS_INFO("eND OF STAND UP");

    while(ros::ok())
    {

        // ROS_INFO("spinOnce");
        // ctrl->loop();
        // ctrl->initMotorParams();

        ros::spinOnce();
        loop_rate.sleep();
    }

    // ros::waitForShutdown();

    // De-Constructors
    delete cmh;
    delete ctrl;
    delete robot;

    return 0;
}
