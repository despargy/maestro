/* Header file for Controller class under RCD*/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/Odometry.h>

#ifndef _GAZEBOINTERPRETER_H_
#define _GAZEBOINTERPRETER_H_


class GazeboInterpreter
{
public:

    ros::NodeHandle *n_adv;
    ros::NodeHandle *n_sub;

    std::string robot_name;
    unitree_legged_msgs::LowState lowState; 
    unitree_legged_msgs::MotorCmd motor_cmd[12];
    bool start_up;

    // pass LowState
    ros::Subscriber servo_sub[12], footForce_sub[4], imu_sub;
    ros::Publisher pub_gazeboLowState_;
    // get n' set LowCmd
    ros::Publisher servo_pub[12];
    ros::Subscriber sub_gazeboLowCmd_;

    const char* sim_lowstate_topic="/gazebo/lowState/state";
    const char* sim_lowcmd_topic="/gazebo/lowCmd/command";
    const char* sim_comcmd_topic="/sim/gazebo/lowCmd/command";

    GazeboInterpreter();
    ~GazeboInterpreter();
    void initGazeboPub();
    void initGazeboSub();

                    /* Callbacks Gazebo*/
    // use: pass LowState
    void imuCallback(const sensor_msgs::Imu & msg);
    void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
    void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
    void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
    void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
    void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
    void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
    void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
    void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
    void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
    void FRfootCallback(const geometry_msgs::WrenchStamped& msg);
    void FLfootCallback(const geometry_msgs::WrenchStamped& msg);
    void RRfootCallback(const geometry_msgs::WrenchStamped& msg);
    void RLfootCallback(const geometry_msgs::WrenchStamped& msg);
    // use: get LowCmd
    void LowCmdsCallback(const unitree_legged_msgs::LowCmd& msg);
};

#endif