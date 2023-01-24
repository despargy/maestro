#include <Robot.h>

namespace RCD
{
                /* CONSTRUCTOR - DE  */

    Robot::Robot()
    {
        std::cout<<"Robot Constructor"<<std::endl;

        // this->motor_state_= new unitree_legged_msgs::MotorState[this->num_motors];
        // this->motor_cmd_ = new unitree_legged_msgs::MotorCmd[this->num_motors];

    }

    Robot::~Robot()
    {
        std::cout<<"Robot De-Constructor"<<std::endl;
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
