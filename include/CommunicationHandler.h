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
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>

#ifndef _COMMUNICATIONHANDLER_H_
#define _COMMUNICATIONHANDLER_H_


namespace RCD
{
    class CommunicationHandler
    {
    public:
        const char* SIM_LOWCMD_TOPIC="/gazebo/lowCmd/command";
        const char* SIM_LOWSTATE_TOPIC="/gazebo/lowState/state";
        const char* SIM_MODELSTATE_TOPIC="/gazebo/model_states";

        const char* REAL_LOWCMD_TOPIC="/low_cmd";
        const char* REAL_LOWSTATE_TOPIC="/low_state";
        const char* REAL_MODELSTATE_TOPIC="/camera/odom/sample"; 

        const char* control_topic="/maestro/ctrl";
        const char* FOOTR_TOPIC="/maestro/foot/rotation";


        const char* lowcmd_topic;
        const char* lowstate_topic;
        const char* modelstate_topic;

        const char* slip_0_topic= "/fr_contact_state";
        const char* slip_1_topic= "/fl_contact_state"; 
        const char* slip_2_topic= "/rr_contact_state";
        const char* slip_3_topic= "/rl_contact_state";
        
        const char* IMU_REAL_EXP_topic= "/contact_probability";


        std::string ns = "/";  // as in group of 'basic.launch'
        int MODELSTATE_ID, NUM_IMUs;
        bool IMU_OK_0, IMU_OK_1, IMU_OK_2, IMU_OK_3; 
        bool SLIP_DETECTION, ADAPT_B, INF, TIPS, WALK;

        std_msgs::Float32MultiArray dat;

        double t;
        float slip[4];
        bool real_experiment_;
        Robot *robot_;
        ros::NodeHandle *nh_main_;
        ros::NodeHandle *nh_cmh_, *nh_slip_;
        ros::Rate *loop_rate;
        // GAZEBO 
        ros::Subscriber sub_CoMState_, sub_LowState_, sub_Control_;
        ros::Subscriber sub_Slip0_, sub_Slip1_, sub_Slip2_, sub_Slip3_;

        ros::Publisher pub_LowCmd_, pub_FootR_;

        CommunicationHandler();
        CommunicationHandler(Robot* robot, ros::NodeHandle* nh_main);
        ~CommunicationHandler();

        void initCommunicationHandler(); // things to inialize after constructor
        void updateRobotState(const unitree_legged_msgs::LowState& msg); // updates the robot's state from ROS UNITREE
        void lowStateCallback(const unitree_legged_msgs::LowState& msg);
        void CoMStateCallback(const gazebo_msgs::ModelStates& msg);
        void RealCoMStateCallback(const nav_msgs::Odometry& msg);
        void sendLowCmd(unitree_legged_msgs::LowCmd& next_low_cmd); 
        void controlCallback(const std_msgs::Bool& msg);
        void sendLowCmdSleep(unitree_legged_msgs::LowCmd& next_low_cmd);

        void lowSlip0Callback(const std_msgs::Float32& msg);
        void lowSlip1Callback(const std_msgs::Float32& msg);
        void lowSlip2Callback(const std_msgs::Float32& msg);
        void lowSlip3Callback(const std_msgs::Float32& msg);
        // function to check init of imus
        bool ALLIMUOK();
        void publishRotation(Eigen::Matrix3d R);

    };

} // namespace RCD

#endif