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
        double maestro_time;

        // KDL::JntArray jnt_pos_, jnt_effort_;//V2
        // KDL::Jacobian jacobian_;//V2
        // KDL::Frame reference_pose_;//V2

        // boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;//V2
        // boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //V2

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
        double STAND_POS[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                                            0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
                   
        Controller();
        Controller( Robot* robot, CommunicationHandler* cmh);
        ~Controller();
        void loadTree();
        void initMotorParams();
        void standUp();
        void moveDesiredQs(double* targetPos, double duration);
        void initLegsControl();
        void solveAllJacobians(KDL::JntArray q_in);
        void getJointQs();
        void initControl();
        void loop();
        void update();

        void setLowCmd(unitree_legged_msgs::LowCmd next_LowCmd);
        unitree_legged_msgs::LowCmd getLowCmd();

    };

}

#endif