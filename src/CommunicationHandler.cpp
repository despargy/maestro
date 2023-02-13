#include <CommunicationHandler.h>

namespace RCD
{
    
    CommunicationHandler::CommunicationHandler()
    {
        std::cout<<"CommunicationHandler Constructor"<<std::endl;
    }
    CommunicationHandler::CommunicationHandler(Robot* robot,  ros::NodeHandle* nh_main)
    {
        std::cout<<"CommunicationHandler Constructor with arguments"<<std::endl;

        this->robot_ = robot;
        this->nh_main_ = nh_main;
        this->nh_cmh_= new ros::NodeHandle;

    }
    CommunicationHandler::~CommunicationHandler()
    {
        std::cout<<"CommunicationHandler De-Constructor"<<std::endl;
    }
    void CommunicationHandler::initCommunicationHandler()
    {
        this->ns = nh_main_->getNamespace().c_str();
        ROS_INFO(" '%s'", this->ns.c_str());
        if (!nh_main_->getParam(this->ns + "/is_simulation", this->is_simulation_)){
            ROS_ERROR("No is_simulation given in namespace: '%s')", nh_main_->getNamespace().c_str());
        }

        if (!nh_main_->getParam(this->ns + "/joint_names", this->joint_names)){
            ROS_ERROR("No joint_names given in namespace: '%s')", nh_main_->getNamespace().c_str());
        }
        for (int i =0 ; i < this->robot_->num_joints ; i++)
        {
            ROS_INFO("in joint_names: ,'%s'", joint_names[i].c_str());
        }
    }
    void CommunicationHandler::initCommunicationHandlerGazebo()
    {
        sub_gazeboLowState_ = nh_cmh_->subscribe(sim_lowstate_topic, 1, &RCD::CommunicationHandler::gazeboLowStateCallback, this);
        pub_gazeboLowCmd_ = nh_cmh_->advertise<unitree_legged_msgs::LowCmd>(sim_lowcmd_topic,1);
    }

                                /* Callbacks */
    void CommunicationHandler::gazeboLowStateCallback(const unitree_legged_msgs::LowState& msg)
    {
        // Cb Low State
        // ROS_INFO("gazeboLowStateCallback()");
        this->updateRobotState(msg); // TODO set barrier ?
    }
    void CommunicationHandler::updateRobotState(const unitree_legged_msgs::LowState& msg)
    {
        // Set Robot  Low State
        this->robot_->setLowState(msg);
    }
    void CommunicationHandler::gazeboSendLowCmd(unitree_legged_msgs::LowCmd& next_low_cmd)
    {
        pub_gazeboLowCmd_.publish(next_low_cmd);
        ros::spinOnce();
        ROS_INFO("Publish at /gazebo/lowCmd/command ");
    }

    // LATER ON set connection with real roobt
    // void CommunicationHandler::setRealConnection()
    // {
    //     ROS_INFO("setRealConnection()");
    //     ROS_INFO("Future use to set connection with robot");
    // }
    // void CommunicationHandler::sendToActualRobot()
    // {
    //     std::cout << "Fix how to inform actual robot for the next task" << std::endl;
    // }


} // namespace CommunicationHandler 
