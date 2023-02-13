/* Header file for Controller class under RCD*/

#include <string.h>
#include <iostream>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h> 
#include <boost/scoped_ptr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <CommunicationHandler.h>
#include <Robot.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <boost/scoped_ptr.hpp> //V2



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

        KDL::JntArray jnt_pos_, jnt_effort_;//V2
        KDL::Jacobian jacobian_;//V2
        KDL::Frame reference_pose_;//V2

        boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;//V2
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //V2

    public:
        CommunicationHandler *cmh_;
        Robot *robot_;
        urdf::Model urdf_model_;
        std::string urdf_file_;
        KDL::Tree robot_tree_; //V1
        
        double STAND_POS[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                                            0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
                                            
        Controller();
        Controller( Robot* robot, CommunicationHandler* cmh);
        ~Controller();
        int initController();
        int loadTree();
        void standUp();
        void initMotorParams();
        void moveDesiredQs(double* targetPos, double duration);
        void loop();
        // void next(double* targetPos);

        void setLowCmd(unitree_legged_msgs::LowCmd next_LowCmd);
        unitree_legged_msgs::LowCmd getLowCmd();

    };

}

#endif