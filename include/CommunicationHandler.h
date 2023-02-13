/* Header file for CommunicationHandler class under RCD*/

#include <iostream>
#include <string.h>
#include <Robot.h>
#include <unitree_legged_msgs/IMU.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/LowCmd.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <maestro/LowCmdRos.h>
#include <maestro/LowStateRos.h>
#include <gazebo_msgs/LinkState.h>
#include <geometry_msgs/WrenchStamped.h>

#ifndef _COMMUNICATIONHANDLER_H_
#define _COMMUNICATIONHANDLER_H_


namespace RCD
{
    class CommunicationHandler
    {
    private:
        const double tick = 0.001;
        std::vector<std::string> joint_names{};

    public:
        const char* sim_lowcmd_topic="/gazebo/lowCmd/command";
        const char* sim_lowstate_topic="/gazebo/lowState/state";

        std::string ns = "/";  // as in group of 'basic.launch'

        double t;
        bool is_simulation_;
        Robot *robot_;
        ros::NodeHandle *nh_main_;
        ros::NodeHandle *nh_cmh_;

        // REAL
        // ros::Subscriber sub_realRobotState_;

        // GAZEBO 
        ros::Subscriber sub_gazeboLowState_;
        ros::Publisher pub_gazeboLowCmd_;

        CommunicationHandler();
        CommunicationHandler(Robot* robot, ros::NodeHandle* nh_main);
        ~CommunicationHandler();

        void initCommunicationHandler(); // things to inialize after constructor
        void initCommunicationHandlerGazebo();
        void updateRobotState(const unitree_legged_msgs::LowState& msg); // updates the robot's state from ROS UNITREE
        void gazeboLowStateCallback(const unitree_legged_msgs::LowState& msg);
        void gazeboSendLowCmd(unitree_legged_msgs::LowCmd& next_low_cmd); 

        // LATER ON
        // void setRealConnection(); // to set set connection with REAL Robot 
        // void sendToActualRobot(); // send cmd to the actual robot


    };

} // namespace RCD

#endif