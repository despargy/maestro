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
// #include <maestro/LowCmdROS.h>
// #include <maestro/LowStateROS.h>

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
        const char* sim_lowcmd_topic="/maestro_gazebo/LowCmd";
        const char* sim_lowstate_topic="/maestro_gazebo/LowState";
        const char* sim_jointstate_topic="/joint_states";

        std::string ns = "/";  // as in group of 'basic.launch'

        double t;
        bool is_simulation_;
        Robot *robot_;
        ros::NodeHandle *nh_main_;
        ros::NodeHandle *nh_cmh_;

        // ros::Subscriber sub_realRobotState_;
        ros::Subscriber sub_gazeboRobotState_;
        ros::Subscriber sub_gazeboJointState_;
        ros::Subscriber sub_gazeboLowCmd_;

        ros::Publisher pub_gazeboLowCmd_;
        ros::Publisher pub_gazeboLowState_;
        ros::Publisher pub_gazeboJointState_;


        CommunicationHandler();
        CommunicationHandler(Robot* robot, ros::NodeHandle* nh_main);
        ~CommunicationHandler();

        // TODO MAY I NEED MORE THAT ONE UPDATE SYNC OR ASYNC?
        void setRealConnection(); // to set set connection with REAL Robot 
        void updateRobotState(const unitree_legged_msgs::LowState & msg); // updates the robot's state from ROS UNITREE
        void sendToActualRobot(); // send cmd to the actual robot
        void initCommunicationHandler(); // things to inialize after constructor
        void gazeboLowStateCallback(const unitree_legged_msgs::LowState & msg);
        void gazeboSendLowCmd(unitree_legged_msgs::LowCmd next_low_cmd); //TODO need & ?
        void gazeboJStoLowState_Cb(const sensor_msgs::JointState & msg);
        void gazeboLowCmdtoJS_Cb(const unitree_legged_msgs::LowCmd & msg);


    };

} // namespace RCD

#endif