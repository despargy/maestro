#include <Robot.h>
namespace RCD
{
                /* CONSTRUCTOR - DE  */

    Robot::Robot()
    {
        ROS_INFO("Robot Constructor");

        this->p_c0 = Eigen::Vector3d::Zero(); // TODO add init values
        // position CoM
        this->p_c = Eigen::Vector3d::Zero();
        this->p_c_update = Eigen::Vector3d::Zero();
        // linear vel CoM
        this->dp_c = Eigen::Vector3d::Zero();
        this->dp_c_update = Eigen::Vector3d::Zero();

        // ori CoM
        this->R_c.resize(3,3);
        this->R_c_update.resize(3,3);
        // ang. vel CoM
        this->w_c = Eigen::Vector3d::Zero();
        this->w_c_update = Eigen::Vector3d::Zero();


        this->F_a.resize(12);
        this->F_c.resize(6);
        this->Gq.resize(6,12);
        this->Gq_sudo.resize(12,6);
        this->gc.resize(6);

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

        // vvvv = w0*Eigen::VectorXd::Ones(12); //as Legs wv_leg init
        // vvvv << 1,1,1    ,1,1,1,     1000,1000,1,        1,1,1;
        // this->W_inv = (vvvv.asDiagonal()).inverse();

        this->g_gravity = 9.80;
        // mass and grav vector are set from CMH

        LegR_frame[0] = Eigen::Matrix3d::Identity();
        LegR_frame[1] = Eigen::Matrix3d::Identity();
        LegR_frame[2] = Eigen::Matrix3d::Identity();
        LegR_frame[3] = Eigen::Matrix3d::Identity();

        g_com = Eigen::Matrix4d::Zero();
        g_com(3,3) = 1;
        
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
        this->p_c_update(0) = com_state.position.x ;
        this->p_c_update(1) = com_state.position.y ;
        this->p_c_update(2) = com_state.position.z ;

        Eigen::Quaterniond cur_c(com_state.orientation.w, com_state.orientation.x, com_state.orientation.y, com_state.orientation.z);
        cur_c.normalize();
        this->R_c_update = cur_c.toRotationMatrix();

    }
    void Robot::setCoMVelocityfromMState(geometry_msgs::Twist com_vel)
    {

        this->dp_c_update(0) = com_vel.linear.x;
        this->dp_c_update(1) = com_vel.linear.y;
        this->dp_c_update(2) = com_vel.linear.z;

        this->w_c_update(0) = com_vel.angular.x;
        this->w_c_update(1) = com_vel.angular.y;
        this->w_c_update(2) = com_vel.angular.z;
    }
    void Robot::setCoMfromCamera(nav_msgs::Odometry camera_pose)
    {
        this->p_c_update(0) = camera_pose.pose.pose.position.x; // SOS dif. frame
        this->p_c_update(1) = camera_pose.pose.pose.position.y;   // SOS dif frame
        this->p_c_update(2) = camera_pose.pose.pose.position.z;

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY( low_state_.imu.rpy[0], low_state_.imu.rpy[1], low_state_.imu.rpy[2] );  
        myQuaternion=myQuaternion.normalize();

        Eigen::Quaterniond cur_c(myQuaternion.getW(), myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ());
        cur_c.normalize();
        this->R_c_update = cur_c.toRotationMatrix();

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
