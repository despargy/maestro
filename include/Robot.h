/* Header file for Robot class under RCD*/

#include <string.h>
#include <iostream>
#include <maestro/Foot.h>
#include <unitree_legged_msgs/IMU.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowState.h>

// #include <model.h>

#ifndef _ROBOT_H_
#define _ROBOT_H_

namespace RCD
{
    
    class Robot
    {
    private:
         
        // unitree_legged_msgs::IMU imu_base_; // imu
        // unitree_legged_msgs::MotorState *motor_state_; // list of 20 motors
        // maestro::Foot foot_force_;

    public:
    
        // unitree_legged_msgs::MotorCmd *motor_cmd_; // list of 20 motors
        unitree_legged_msgs::LowState low_state_; 

        const int num_joints = 12; 
        const int num_motors = 20;

        std::string robot_name_ ;

        Robot(/* args */);
        ~Robot();

        // void setImuBase(unitree_legged_msgs::IMU imu);
        // void setFoot(maestro::Foot foot_force);
        // void setMotorState(unitree_legged_msgs::MotorState* motor_state);
        // void setMotorCmd(unitree_legged_msgs::MotorCmd* motor_cmd);
        void setLowState(unitree_legged_msgs::LowState low_state);

        // maestro::Foot getFoot();
        // unitree_legged_msgs::IMU getImuBase();
        // unitree_legged_msgs::MotorState* getMotorState();
        // unitree_legged_msgs::MotorCmd* getMotorCmd();
        unitree_legged_msgs::LowState getLowState();

    };  

} // namespace RCD

#endif