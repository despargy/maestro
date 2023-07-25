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
namespace RCD
{
    class Controller
    {
    private:
        std::string ns;
        unitree_legged_msgs::LowCmd next_LowCmd_; 
        int n_leg;
        double kp,ko,kv,b_coef, alpha;
        double d_tv, tv, t_real, dt, t_swing ;
        // double start_swing[3], target_swing[3];
        double start_swing[3] = {0.0, 0.67, -1.5};
        // double target_swing[3] = {0.0,  +1.5 - 0.3 , -M_PI + 0.4};
        double target_swing[3] = {0.0,  0.67 + 0.80 , -M_PI + 0.4};

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
        void computeWeights(double dt);
        void forceTrasform();
        void computeBeta_t();
        void initDataHandler();
        void firstCommandForRealRobot();
        void updateCoM();
        void updateVelocityCoM();
        void computeWeightsInfinity(double dt, double time_now);
        void setNewCmdSwing(double time_now);

  };

}

#endif