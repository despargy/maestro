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

        // this->pub_nextLowCmd_ = this->nh_cmh_->advertise<unitree_legged_msgs::LowCmd>(this->lowcmd_topic,1) ;

        // lowState_pub = nh_main_.advertise<unitree_legged_msgs::LowState>("/go1_gazebo/lowState/state", 1);

        // //SUPER SUPER SOS TODO 
        // if( !this->is_simulation_) 
        //     this->setRealConnection();
        // else
        // {
        //     sub_simRobotState_ = nh_cmh_->subscribe("/go1_gazebo/lowState/state", 1, &CommunicationHandler::gazeboLowStateCallback, this);
        // }
    }
    void CommunicationHandler::gazeboLowStateCallback(const unitree_legged_msgs::LowState & msg)
    {
        // TODO set barrier ?
        ROS_INFO("gazeboLowStateCallback()");
        this->updateRobotState(msg);
    }
        // TODO MAY I NEED MORE THAT ONE UPDATE SYNC OR ASYNC?
    void CommunicationHandler::updateRobotState(const unitree_legged_msgs::LowState & msg)
    {
        // Low State
        this->robot_->setLowState(msg);
        ROS_INFO("updateRobotState is called");
    }
    //CHECKED until here

    void CommunicationHandler::gazeboSendLowCmd(unitree_legged_msgs::LowCmd next_low_cmd)
    {
        this->pub_gazeboLowCmd_.publish(next_low_cmd);
    }

    //TODO set connection with real roobt
    void CommunicationHandler::setRealConnection()
    {
        ROS_INFO("setRealConnection()");
        ROS_INFO("Future use to set connection with robot");
    }
    void CommunicationHandler::sendToActualRobot()
    {
        std::cout << "Fix how to inform actual robot for the next task" << std::endl;
    }
    void CommunicationHandler::gazeboJStoLowState_Cb(const sensor_msgs::JointState & msg)
    {
        ROS_INFO("tHIS IS ONE JSTATE '%f'", msg.position[0]);
        unitree_legged_msgs::LowState ls;
        // ls.head = ros::time
        ls.footForce = {100, 100, 100, 100};
        for (int m = 0; m < 12;m++)
        {
            ls.motorState[m].q = msg.position[m];

        }
        // ls.head
        this->pub_gazeboLowState_.publish(ls);
    }
    void CommunicationHandler::gazeboLowCmdtoJS_Cb(const unitree_legged_msgs::LowCmd & msg)
    {
        ROS_INFO(" in gazeboLowCmdtoJS_Cb" );

    }

} // namespace CommunicationHandler 
