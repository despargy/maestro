#include <Robot.h>
namespace RCD
{
                /* CONSTRUCTOR - DE  */

    Robot::Robot()
    {
        ROS_INFO("Robot Constructor");
        // this->motor_state_= new unitree_legged_msgs::MotorState[this->num_motors];
        // this->motor_cmd_ = new unitree_legged_msgs::MotorCmd[this->num_motors];
        this->p_c = Eigen::Vector3d::Zero();
        this->com_vel_linear = Eigen::Vector3d::Zero();
        this->com_vel_ang= Eigen::Vector3d::Zero();
        this->R_c.resize(3,3);
        this->F_a.resize(12);
        this->F_c.resize(12);
        this->Gq.resize(6,12);
        this->Gq_sudo.resize(12,6);
        this->gc.resize(6);
        //init Gq
        this->Gq.block(0,0,3,3) =  Eigen::Matrix3d::Identity();
        this->Gq.block(0,3,3,3) =  Eigen::Matrix3d::Identity();
        this->Gq.block(0,6,3,3) =  Eigen::Matrix3d::Identity();
        this->Gq.block(0,9,3,3) =  Eigen::Matrix3d::Identity();

        // init W once
        Eigen::VectorXd wv;
        wv.resize(12);
        wv << 20,20,1,1,1,1,1,1,1,1,1,1; //TODO
        this->W_inv = (wv.asDiagonal()).inverse();

        this->mass = 13.1; // if is real exp. cmh changes it to 12.0kg 
        this->g_gravity = 10.0;
        this->gc << 0,0,this->mass*this->g_gravity,0,0,0;
        this->KEEP_CONTROL = true;

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
        this->p_c(0) = com_state.position.x;
        this->p_c(1) = com_state.position.y;
        this->p_c(2) = com_state.position.z;

        // this->com_q.x() = com_state.orientation.x;
        // this->com_q.y() = com_state.orientation.y;
        // this->com_q.z() = com_state.orientation.z;
        // this->com_q.w() = com_state.orientation.w;
        // this->com_q.normalize();
        // this->R_c = this->com_q.toRotationMatrix();
        // std::cout << "R_c=" << std::endl << this->R_c << std::endl;

        Eigen::Quaterniond cur_c(com_state.orientation.w, com_state.orientation.x, com_state.orientation.y, com_state.orientation.z);
        cur_c.normalize();
        this->R_c = cur_c.toRotationMatrix();
        // std::cout << "R_c=" << std::endl << this->R_c << std::endl;

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
