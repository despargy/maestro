/* Header file for Controller class under RCD*/

#include <string.h>
#include <iostream>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h> 
#include <boost/scoped_ptr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <CommunicationHandler.h>
#include <Robot.h>

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

    public:
        CommunicationHandler *cmh_;
        Robot *robot_;
        urdf::Model urdf_model_;
        std::string urdf_file_;
        KDL::Tree robot_tree_;

        Controller();
        Controller( Robot* robot, CommunicationHandler* cmh);
        ~Controller();
        int initController();
        int loadTree();
        void standUp();
        void initMotorParams();
        void moveDesiredQs(double* targetPos, double duration);
        void loop();

        void setLowCmd(unitree_legged_msgs::LowCmd next_LowCmd);
        unitree_legged_msgs::LowCmd getLowCmd();

    };

}

#endif