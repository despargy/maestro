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
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#ifndef _COMMUNICATIONHANDLER_H_
#define _COMMUNICATIONHANDLER_H_


namespace RCD
{
    class CommunicationHandler
    {
    private:
        const double tick = 0.001;

    public:
        const char* SIM_LOWCMD_TOPIC="/gazebo/lowCmd/command";
        const char* SIM_LOWSTATE_TOPIC="/gazebo/lowState/state";
        const char* SIM_MODELSTATE_TOPIC="/gazebo/model_states";

        const char* REAL_LOWCMD_TOPIC="/low_cmd";
        const char* REAL_LOWSTATE_TOPIC="/low_state";
        const char* REAL_MODELSTATE_TOPIC="/TODO_COM"; //TODO

        const char* control_topic="/maestro/ctrl";

        const char* lowcmd_topic;
        const char* lowstate_topic;
        const char* modelstate_topic;

        
        std::string ns = "/";  // as in group of 'basic.launch'
        int MODELSTATE_ID; 

        double t;
        bool real_experiment_;
        Robot *robot_;
        ros::NodeHandle *nh_main_;
        ros::NodeHandle *nh_cmh_;
        // ros::Rate *loop_rate;

        // GAZEBO 
        ros::Subscriber sub_CoMState_, sub_LowState_, sub_Control_;
        ros::Publisher pub_LowCmd_;

        CommunicationHandler();
        CommunicationHandler(Robot* robot, ros::NodeHandle* nh_main);
        ~CommunicationHandler();

        void initCommunicationHandler(); // things to inialize after constructor
        void updateRobotState(const unitree_legged_msgs::LowState& msg); // updates the robot's state from ROS UNITREE
        void lowStateCallback(const unitree_legged_msgs::LowState& msg);
        void CoMStateCallback(const gazebo_msgs::ModelStates& msg);
        void RealCoMStateCallback(const gazebo_msgs::ModelStates& msg); // TODO SOSOSOSOSOSSO
        void sendLowCmd(unitree_legged_msgs::LowCmd& next_low_cmd); 
        void controlCallback(const std_msgs::Bool& msg);
        void sendLowCmdSleep(unitree_legged_msgs::LowCmd& next_low_cmd);

    };

} // namespace RCD

#endif