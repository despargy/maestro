#include <Robot.h>
namespace RCD
{
                /* CONSTRUCTOR - DE  */

    Robot::Robot()
    {
        ROS_INFO("Robot Constructor");

        this->p_c = Eigen::Vector3d::Zero();

        this->R_c.resize(3,3);
        this->F_a.resize(12);
        this->F_c.resize(12);
        this->Gq.resize(6,12);
        this->Gq_sudo.resize(12,6);
        this->gc.resize(6);

        this->mass = 13.1; // if is real exp. cmh changes it to 12.0kg 
        this->g_gravity = 9.80;
        this->gc << 0,0,this->mass*this->g_gravity,0,0,0;
        this->KEEP_CONTROL = true;

        this->H_c.resize(6,6);
        this->C_c.resize(6,6);
        // init I
        Eigen::Vector3d ii(0.02, 0.07, 0.08); //inetria tensor
        this->I = ii.asDiagonal();
        // init H_c
        this->H_c.block(0,0,3,3) = this->mass*Eigen::Matrix3d::Identity();
        this->H_c.block(3,3,3,3) = Eigen::Matrix3d::Zero(); // will later be set based on Rc*Ic*Rc.t() 
        // init C_c
        this->C_c.block(0,0,3,3) = Eigen::Matrix3d::Zero();
        this->C_c.block(3,3,3,3) = Eigen::Matrix3d::Zero();
        //init Gq
        this->Gq.block(0,0,3,3) =  Eigen::Matrix3d::Identity();
        this->Gq.block(0,3,3,3) =  Eigen::Matrix3d::Identity();
        this->Gq.block(0,6,3,3) =  Eigen::Matrix3d::Identity();
        this->Gq.block(0,9,3,3) =  Eigen::Matrix3d::Identity();
        // init W once
        vvvv.resize(12);
         vvvv << 1,1,1,   1,1,1,    1000,1000,1,    1,1,1; //TODO as Legs wv_leg init
        // vvvv << 10,10,10,10,10,10,10,10,10,10,10,10; //TODO as Legs wv_leg init
        this->W_inv = (vvvv.asDiagonal()).inverse();
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
    void Robot::setCoMfromMState(geometry_msgs::Pose com_state)
    {
        this->p_c(0) = com_state.position.x;
        this->p_c(1) = com_state.position.y;
        this->p_c(2) = com_state.position.z;

        Eigen::Quaterniond cur_c(com_state.orientation.w, com_state.orientation.x, com_state.orientation.y, com_state.orientation.z);
        cur_c.normalize();
        this->R_c = cur_c.toRotationMatrix();

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
