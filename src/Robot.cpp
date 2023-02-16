#include <Robot.h>
namespace RCD
{
                /* CONSTRUCTOR - DE  */

    Robot::Robot()
    {
        ROS_INFO("Robot Constructor");
        // this->motor_state_= new unitree_legged_msgs::MotorState[this->num_motors];
        // this->motor_cmd_ = new unitree_legged_msgs::MotorCmd[this->num_motors];
        com_pos = Eigen::Vector3d::Zero();
        com_vel_linear = Eigen::Vector3d::Zero();
        com_vel_ang= Eigen::Vector3d::Zero();
    }

    Robot::~Robot()
    {
        ROS_INFO("Robot De-Constructor");
    }

                 /* SET FUNCTIONS */
    // void Robot::setImuBase(unitree_legged_msgs::IMU imu)
    // {
    //     this->imu_base_ = imu;
    // }
    // void Robot::setFoot(maestro::Foot foot_force)
    // {
    //     this->foot_force_ = foot_force;
    // }
    // void Robot::setMotorState(unitree_legged_msgs::MotorState* motor_state)
    // {
    //     this->motor_state_ = motor_state;
    // }
    // void Robot::setMotorCmd(unitree_legged_msgs::MotorCmd* motor_cmd)
    // {
    //     this->motor_cmd_ = motor_cmd;
    // }
    void Robot::setLowState(unitree_legged_msgs::LowState low_state)
    {
        this->low_state_ = low_state;
    }
    void Robot::setCoMfromMState(geometry_msgs::Pose com_state, geometry_msgs::Twist com_state_dot)
    {
        this->com_pos(0) = com_state.position.x;
        this->com_pos(1) = com_state.position.y;
        this->com_pos(2) = com_state.position.z;

        this->com_q.x() = com_state.orientation.x;
        this->com_q.y() = com_state.orientation.y;
        this->com_q.z() = com_state.orientation.z;
        this->com_q.w() = com_state.orientation.w;
        this->com_q.normalize();

        this->com_vel_linear(0) = com_state_dot.linear.x;
        this->com_vel_linear(1) = com_state_dot.linear.y;
        this->com_vel_linear(2) = com_state_dot.linear.z;

        this->com_vel_ang(0) = com_state_dot.angular.x;
        this->com_vel_ang(1) = com_state_dot.angular.y;
        this->com_vel_ang(2) = com_state_dot.angular.z;
    }

                /* GET FUNCTIONS */
    // unitree_legged_msgs::IMU Robot::getImuBase()
    // {
    //     return this->imu_base_;
    // }
    // maestro::Foot Robot::getFoot()
    // {
    //     return this->foot_force_;
    // }
    // unitree_legged_msgs::MotorState* Robot::getMotorState()
    // {
    //     return motor_state_;
    // }
    // unitree_legged_msgs::MotorCmd* Robot::getMotorCmd()
    // {
    //     return motor_cmd_;
    // }
     unitree_legged_msgs::LowState Robot::getLowState()
    {
        return low_state_;
    }   

} // namespace Robot
