/* Header file for Controller class under RCD*/
// FL -> 1  // FR -> 0
// RL -> 3  // RR -> 2

#include <string.h>
#include <iostream>
#include <urdf/model.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <CommunicationHandler.h>
#include <Robot.h>
#include <Leg.h>
#include <chrono>
#include <ctime>
#include <Math.h>

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
        Leg *leg_mng;
        int n_leg;
        double kp,ko,kv,b_coef;
    public:
        // Chrono
        std::chrono::time_point<std::chrono::system_clock> time_start, time_end, time_cur;
        std::chrono::duration<double> time_elapsed;
        // Maestro obj.
        CommunicationHandler *cmh_;
        Robot *robot_;
        urdf::Model urdf_model_;
        std::string urdf_file_;
        KDL::Tree robot_kin;
        Math math_lib;

        Controller();
        Controller( Robot* robot, CommunicationHandler* cmh);
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
        void updateControlLaw();
        void setNewCmd();
        void setMotorModeGains();
        void gravComp();
        void startingPose();
  };

}

#endif