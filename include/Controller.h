/* Header file for Controller class under RCD*/
// FL -> 1  // FR -> 0
// RL -> 3  // RR -> 2

#include <string.h>
#include <iostream>
#include <urdf/model.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <CommunicationHandler.h>
#include <DataHandler.h>
#include <Robot.h>
#include <Leg.h>
#include <chrono>
#include <ctime>
#include <Math.h>
#include <vector>

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)


const int PH_TARGET = 0;
const int PH_SWING = 1;

namespace RCD
{
    class Controller
    {
    private:
        std::string ns;
        unitree_legged_msgs::LowCmd next_LowCmd_; 
        int n_leg;
        int* vp_order;
        int* free_gait;
        double kp, ko, kv, ki, b_coef, alpha;
        double d_tv, tv, t_real, dt, t_swing, t0_phase, t_phase, t_to_use, swing_t_slot ;
        double A,b, t0_superG, t0_swing, t_half_swing;
        float freq_swing;
        double q_start_swing[3] = {0.0, 0.67, -1.5};
        double q_target_swing[3] = {0.0,  0.67 + 0.80 , -M_PI + 0.4};
        Eigen::Vector3f d_traj_0frame, d_traj_0frame_old, d_vel_0frame;

        Eigen::Vector4f d_tip_pos;
        Eigen::Vector3f d_tip_vel;
        
        Eigen::Vector3d e_p_int, e_o_int;
        Eigen::VectorXd pid_out;
        Eigen::Vector3d p_d0, ddp_d, RcRdTwd, pbc;
        Eigen::Matrix3d R_d_0, dR_d, Re;
        Eigen::Quaterniond Q_0;
        Eigen::MatrixXd Gbc;
        Eigen::AngleAxisd ang;
        // vector to help with eq. 11
        Eigen::VectorXd fcontrol1,fcontrol2,fcontrol3; 
        int LOC_STATE;
    public:

        // Maestro obj.
        CommunicationHandler *cmh_;
        Robot *robot_;
        DataHandler *data_handler_;
        Leg *leg_mng;
        urdf::Model urdf_model_;
        std::string urdf_file_;
        KDL::Tree robot_kin;
        Math math_lib;


        // Error variables
        Eigen::Vector3d e_p, e_o;
        Eigen::VectorXd e_v;
        Eigen::Vector3d p_d;
        Eigen::Matrix3d R_d;
        Eigen::Vector3d w_d;
        Eigen::Vector3d dp_d;
        Eigen::Matrix4d g_d; // for locomotion only

        Controller();
        Controller( Robot* robot, CommunicationHandler* cmh, DataHandler* data_handler);
        ~Controller();
        void loadTree();
        void initMotorParamsHard();
        void standUp();
        void moveDesiredQs(double* targetPos, double duration);
        void initLegsControl();
        void getLegQF();
        void initControl();
        void loop();
        void updateLegs();
        void solveJacP();
        void computeSudoGq();
        void updateControlLaw(Eigen::Vector3d w_com);
        void setNewCmd();
        void setMaestroMotorGains();
        void gravComp();
        void startingPose();
        void computeWeights();
        void forceTrasform();
        void computeBeta_t();
        void initDataHandler();
        void firstCommandForRealRobot();
        void updateCoM();
        void updateVelocityCoM();
        void computeWeightsInfinity();
        void setNewCmdRise();
        void locomotion_loop();
        void initTarget();
        void setPhaseTarget();
        void updateCoMTipsWorld();
        void setMaestroMotorGainsWalk();
        void computeWeightsSwing();
        void initLocomotion();
        void positionError(), positionErrorTarget();
        void velocityError(), velocityErrorTarget();
        void PIDwithSat();
        void fComputations(), fComputationsTarget();
        void getTrajD();
        void setNewCmdSwing();
        void inverseTip();
        void setQTips();
        void CLIK(Eigen::Vector3f pd_0frame_, Eigen::Vector3f dpd_0frame_);
        void setMaestroMotorGainsWalk_0(), setMaestroMotorGainsWalk_1(), setMaestroMotorGainsWalk_2();

  };

}

#endif