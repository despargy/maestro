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
    // Init ROS node TODO
    ros::init(argc, argv, "main_handler");
    ros::NodeHandle nh;
    ros::Rate loop_rate(500);

    // Construct a Robot, CommunicationHandler, Controller
    RCD::Robot* robot = new RCD::Robot();
    RCD::CommunicationHandler* cmh = new RCD::CommunicationHandler(robot, &nh);
    RCD::Controller* ctrl = new RCD::Controller(robot, cmh);

    // Initialize cmh, subscribe to "Simulated Robot"
    cmh->initCommunicationHandler();
    
    //SUPER SUPER SOS TODO 
    if( !cmh->is_simulation_) 
        cmh->setRealConnection();
    else
    {
        // subscribe to Maestro LowState
        cmh->sub_gazeboRobotState_ = (cmh->nh_cmh_)->subscribe(cmh->sim_lowstate_topic, 1, &RCD::CommunicationHandler::gazeboLowStateCallback, cmh);
        // publish to Gazebo LowCmd 
        cmh->pub_gazeboLowCmd_ = cmh->nh_cmh_->advertise<unitree_legged_msgs::LowCmd>(cmh->sim_lowcmd_topic, 1); //
        
        
        // subscribe to Gazebo JointState and publish to Maestro LowState
        cmh->sub_gazeboJointState_= (cmh->nh_cmh_)->subscribe(cmh->sim_jointstate_topic, 1, &RCD::CommunicationHandler::gazeboJStoLowState_Cb, cmh);
        cmh->pub_gazeboLowState_ = cmh->nh_cmh_->advertise<unitree_legged_msgs::LowState>(cmh->sim_lowstate_topic, 1); //

        // subscribe to Maestro LowCmd and publish to Gazebo Gazebo 
        cmh->sub_gazeboLowCmd_ = (cmh->nh_cmh_)->subscribe(cmh->sim_lowcmd_topic, 1, &RCD::CommunicationHandler::gazeboLowCmdtoJS_Cb, cmh);
        cmh->pub_gazeboJointState_ = cmh->nh_cmh_->advertise<sensor_msgs::JointState>(cmh->sim_jointstate_topic, 1); //

    }
    // Initialize Controller
    if( ctrl->initController()) ROS_ERROR("Fail to Inialize Controller");
    else ROS_INFO("Successfully Inialize Controller");
    

    // ROS_INFO("name_space: ,'%s'", nh.getNamespace().c_str());
    // ROS_INFO("joint_name: ,'%s'", joint_name.c_str());
    // ctrl->standUp();

    while(ros::ok())
    {
        
        ros::spinOnce();
        loop_rate.sleep();
    }


    // De-Constructors
    delete cmh;
    delete ctrl;
    delete robot;

    return 0;
}
