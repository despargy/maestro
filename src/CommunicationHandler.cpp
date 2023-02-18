#include <CommunicationHandler.h>

namespace RCD
{
    
    CommunicationHandler::CommunicationHandler()
    {
        std::cout<<"CommunicationHandler Constructor"<<std::endl;
    }
    CommunicationHandler::CommunicationHandler(Robot* robot,  ros::NodeHandle* nh_main)
    {
        ROS_INFO("CommunicationHandler Constructor with arguments");
        this->robot_ = robot;
        this->nh_main_ = nh_main;
        this->nh_cmh_= new ros::NodeHandle;
        this->MODELSTATE_ID = 4; //do not cure unless is simulation = not real+experiment
    }
    CommunicationHandler::~CommunicationHandler()
    {
        ROS_INFO("CommunicationHandler De-Constructor");
    }
    void CommunicationHandler::initCommunicationHandler()
    {
        // Get params
        this->ns = nh_main_->getNamespace().c_str();
        ROS_INFO(" '%s'", this->ns.c_str());
        if (!nh_main_->getParam(this->ns + "/real_experiment", this->real_experiment_))
        {
            ROS_ERROR("Param /real_experiment missing from namespace: '%s'", nh_main_->getNamespace().c_str());
        }
        // Set info to 'Robot' obj.
        if (!nh_main_->getParam(this->ns + "/joint_names", this->robot_->joint_names))
        {
            ROS_ERROR("Param /joint_names missing from namespace: '%s' ", nh_main_->getNamespace().c_str());
        }
        // Select topics if is real or simulation
        if (!this->real_experiment_)
        {
            this->lowcmd_topic = this->SIM_LOWCMD_TOPIC; // Select the simulation topics
            this->lowstate_topic = this->SIM_LOWSTATE_TOPIC; // Select the simulation topics
            this->modelstate_topic = this->SIM_MODELSTATE_TOPIC; // Select the simulation topics // diff. Cb
            sub_CoMState_ = nh_cmh_->subscribe(this->modelstate_topic, 1, &RCD::CommunicationHandler::CoMStateCallback, this); // diff. Cb
        }
        else
        {
            this->robot_->mass = 12.0; // do not forget it
            this->lowcmd_topic = this->REAL_LOWCMD_TOPIC; // Select the real topics
            this->lowstate_topic = this->REAL_LOWSTATE_TOPIC; // Select the real topics
            this->modelstate_topic = this->REAL_MODELSTATE_TOPIC; // Select the simulation topics // diff. Topic
            sub_CoMState_ = nh_cmh_->subscribe(this->modelstate_topic, 1, &RCD::CommunicationHandler::RealCoMStateCallback, this); // diff. Cb
            std::cout<<"TODO set real connection with robot and CoM state topic"<<std::endl;
        }
        // General sub-pus to /LowCmds and /LowState
        sub_LowState_ = nh_cmh_->subscribe(this->lowstate_topic, 1, &RCD::CommunicationHandler::lowStateCallback, this);
        pub_LowCmd_ = nh_cmh_->advertise<unitree_legged_msgs::LowCmd>(this->lowcmd_topic,1);
        sub_Control_ = nh_cmh_->subscribe(this->control_topic, 1, &RCD::CommunicationHandler::controlCallback, this);
    }
                                /* Callbacks */
    void CommunicationHandler::controlCallback(const std_msgs::Bool& msg)
    {
        // Cb CoM State 
        // std::cout<<msg<<std::endl;
        this->robot_->KEEP_CONTROL = msg.data; 
    }    
    void CommunicationHandler::CoMStateCallback(const gazebo_msgs::ModelStates& msg)
    {
        // Cb CoM State 
        this->robot_->setCoMfromMState(msg.pose[MODELSTATE_ID], msg.twist[MODELSTATE_ID]); 
    }
    // TODO real com pos vel
    void CommunicationHandler::RealCoMStateCallback(const gazebo_msgs::ModelStates& msg)
    {
        // Cb CoM State REAL
        // this->robot_->setCoMfromMState(msg.pose[MODELSTATE_ID], msg.twist[MODELSTATE_ID]); 
        std::cout<<"msg state REAL TODO\n"<<msg<<std::endl;
    }
    void CommunicationHandler::lowStateCallback(const unitree_legged_msgs::LowState& msg)
    {
        // Cb Low State
        this->updateRobotState(msg); 
    }
    void CommunicationHandler::updateRobotState(const unitree_legged_msgs::LowState& msg)
    {
        // Set Robot Low State
        this->robot_->setLowState(msg);
    }
    void CommunicationHandler::sendLowCmd(unitree_legged_msgs::LowCmd& next_low_cmd)
    {
        pub_LowCmd_.publish(next_low_cmd);
        ros::spinOnce();
    }


} // namespace CommunicationHandler 
