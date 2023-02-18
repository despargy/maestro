/* Header file for Robot class under RCD*/

#include <string.h>
#include <iostream>
#include <maestro/Foot.h>
#include <unitree_legged_msgs/IMU.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <ros/ros.h>

// #include <model.h>

#ifndef _ROBOT_H_
#define _ROBOT_H_

namespace RCD
{
    
    class Robot
    {
    // private:
        // unitree_legged_msgs::IMU imu_base_; // imu
        // unitree_legged_msgs::MotorState *motor_state_; // list of 20 motors
        // maestro::Foot foot_force_;
    public:
        std::string robot_name_ ;
        std::vector<std::string> joint_names{}; // init by communication handler
        int num_joints;  // init by communication handler
        // int num_motors = 20;
        double mass, g_gravity;
        // unitree_legged_msgs::MotorCmd *motor_cmd_; // list of 20 motors
        unitree_legged_msgs::LowState low_state_; 
        Eigen::Vector3d p_c, com_vel_linear, com_vel_ang;
        // Eigen::Quaterniond com_q;
        Eigen::VectorXd F_a, F_c, gc;
        Eigen::MatrixXd R_c, Gq, Gq_sudo;
        Eigen::DiagonalMatrix<double,12> W_inv;
        Robot(/* args */);
        ~Robot();

        // void setImuBase(unitree_legged_msgs::IMU imu);
        // void setFoot(maestro::Foot foot_force);
        // void setMotorState(unitree_legged_msgs::MotorState* motor_state);
        // void setMotorCmd(unitree_legged_msgs::MotorCmd* motor_cmd);
        void setLowState(unitree_legged_msgs::LowState low_state);
        void setCoMfromMState(geometry_msgs::Pose com_state, geometry_msgs::Twist com_state_dot);

        // maestro::Foot getFoot();
        // unitree_legged_msgs::IMU getImuBase();
        // unitree_legged_msgs::MotorState* getMotorState();
        // unitree_legged_msgs::MotorCmd* getMotorCmd();
        unitree_legged_msgs::LowState getLowState();

    };  

} // namespace RCD

#endif