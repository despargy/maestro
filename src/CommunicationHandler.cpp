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
        this->nh_slip_= new ros::NodeHandle;
        // this->loop_rate = new ros::Rate(5000);
        this->MODELSTATE_ID = 4; //do not cure unless is simulation = not real+experiment
        
        this->IMU_OK_0 = false;
        this->IMU_OK_1 = false;
        this->IMU_OK_2 = false;
        this->IMU_OK_3 = false;

        this->slip[0] = 1.0;
        this->slip[1] = 1.0;
        this->slip[2] = 1.0;
        this->slip[3] = 1.0;

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
        if (!nh_main_->getParam(this->ns + "/num_imus", this->NUM_IMUs))
        {
            ROS_ERROR("Param /num_imus missing from namespace: '%s'", nh_main_->getNamespace().c_str());
        }
        // Set info to 'Robot' obj.
        if (!nh_main_->getParam(this->ns + "/joint_names", this->robot_->joint_names))
        {
            ROS_ERROR("Param /joint_names missing from namespace: '%s' ", nh_main_->getNamespace().c_str());
        }
        if (!this->nh_main_->getParam( this->ns + "/slip_detection", this->SLIP_DETECTION)){
            ROS_ERROR("No SLIP_DETECTION given in namespace: '%s')", this->nh_main_->getNamespace().c_str());
        }
        // Select topics if is real or simulation
        if (!this->real_experiment_)
        {
            // select robot mass and grav. vector
            this->robot_->mass = 13.1; // if is real exp. cmh changes it to 12.0kg 
            this->robot_->gc << 0,0,this->robot_->mass*this->robot_->g_gravity,0,0,0;
            // select topics
            this->lowcmd_topic = this->SIM_LOWCMD_TOPIC; // Select the simulation topics
            this->lowstate_topic = this->SIM_LOWSTATE_TOPIC; // Select the simulation topics
            this->modelstate_topic = this->SIM_MODELSTATE_TOPIC; // Select the simulation topics // diff. Cb
            this->sub_CoMState_ = nh_cmh_->subscribe(this->modelstate_topic, 1, &RCD::CommunicationHandler::CoMStateCallback, this); // diff. Cb
            if(this->SLIP_DETECTION)
            {
                this->sub_Slip0_ = nh_slip_->subscribe(this->slip_0_topic, 1, &RCD::CommunicationHandler::lowSlip0Callback, this);
                this->sub_Slip1_ = nh_slip_->subscribe(this->slip_1_topic, 1, &RCD::CommunicationHandler::lowSlip1Callback, this);
                this->sub_Slip2_ = nh_slip_->subscribe(this->slip_2_topic, 1, &RCD::CommunicationHandler::lowSlip2Callback, this);
                this->sub_Slip3_ = nh_slip_->subscribe(this->slip_3_topic, 1, &RCD::CommunicationHandler::lowSlip3Callback, this);

            }
        }
        else
        {
            // select robot mass and grav. vector
            this->robot_->mass = 12.0; 
            this->robot_->gc << 0,0,this->robot_->mass*this->robot_->g_gravity,0,0,0;
            // select topics
            this->lowcmd_topic = this->REAL_LOWCMD_TOPIC; // Select the real topics
            this->lowstate_topic = this->REAL_LOWSTATE_TOPIC; // Select the real topics
            this->modelstate_topic = this->REAL_MODELSTATE_TOPIC; // Select the simulation topics // diff. Topic
            this->sub_CoMState_ = nh_cmh_->subscribe(this->modelstate_topic, 1, &RCD::CommunicationHandler::RealCoMStateCallback, this); // diff. Cb
            std::cout<<"TODO set real connection with robot and CoM state topic"<<std::endl;
        }
        // General sub-pus to /LowCmds and /LowState
        sub_LowState_ = nh_cmh_->subscribe(this->lowstate_topic, 1, &RCD::CommunicationHandler::lowStateCallback, this);
        pub_LowCmd_ = nh_cmh_->advertise<unitree_legged_msgs::LowCmd>(this->lowcmd_topic,1);
        sub_Control_ = nh_cmh_->subscribe(this->control_topic, 1, &RCD::CommunicationHandler::controlCallback, this);
    }
                                /* Callbacks */
    void CommunicationHandler::lowSlip0Callback(const std_msgs::Float32& msg)
    {
        this->IMU_OK_0 = true;
        this->slip[0] = msg.data; 
    }   
    void CommunicationHandler::lowSlip1Callback(const std_msgs::Float32& msg)
    {
        this->IMU_OK_1 = true;
        this->slip[1] = msg.data; 
    } 
    void CommunicationHandler::lowSlip2Callback(const std_msgs::Float32& msg)
    {
        this->IMU_OK_2 = true;
        this->slip[2] = msg.data; 
    } 
    void CommunicationHandler::lowSlip3Callback(const std_msgs::Float32& msg)
    {
        this->IMU_OK_3 = true;
        this->slip[3] = msg.data; 
    } 
    void CommunicationHandler::controlCallback(const std_msgs::Bool& msg)
    {
        // Cb CoM State 
        this->robot_->KEEP_CONTROL = msg.data; 
    }    
    void CommunicationHandler::CoMStateCallback(const gazebo_msgs::ModelStates& msg)
    {
        // Cb CoM State 
        this->robot_->setCoMfromMState(msg.pose[MODELSTATE_ID]); 
    }
    // TODO real com pos vel
    void CommunicationHandler::RealCoMStateCallback(const gazebo_msgs::ModelStates& msg)
    {
        // Cb CoM State REAL
        // this->robot_->setCoMfromMState(msg.pose[MODELSTATE_ID], msg.twist[MODELSTATE_ID]); 
        std::cout<<"SOSOSOS COM msg state REAL TODO\n"<<msg<<std::endl;
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
    bool CommunicationHandler::ALLIMUOK()
    {
        if ((this->IMU_OK_0? 1:0) + (this->IMU_OK_1? 1:0) + (this->IMU_OK_2? 1:0) + (this->IMU_OK_3? 1:0)  == this->NUM_IMUs)
                return true;
        return false;
    }
} // namespace CommunicationHandler 
