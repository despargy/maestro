#include <Controller.h>

namespace RCD
{
                /* CONSTRUCTOR - DE  */

    Controller::Controller()
    {
        std::cout<<"Controller Constructor"<<std::endl;
    }
    Controller::Controller(Robot* robot, CommunicationHandler* cmh)
    {
        ROS_INFO("Controller Constructor");
        this->cmh_ = cmh ;
        this->robot_ = robot;
        this->ns = this->cmh_->ns;
        if (!this->cmh_->nh_main_->getParam( this->ns + "/urdf_file_path", this->urdf_file_)){
            ROS_ERROR("No is_simulation given in namespace: '%s')", this->cmh_->nh_main_->getNamespace().c_str());
        }
    }
    Controller::~Controller()
    {
        std::cout<<"Controller De-Constructor"<<std::endl;
    }
    int Controller::initController()
    {
        int status = loadTree(); // state 0 means OK

        KDL::Chain chain;
        bool exit_value = robot_tree_.getChain("FR_calf_joint","FR_hip_joint",chain);
        if (exit_value)
        {
            std::cout<<"getChain TRUE"<<std::endl;
        }
        else
        {
            std::cout<<"getChain FALSE "<<std::endl;
        }

        initMotorParams();   
        // std::cout<<chain<<std::endl;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
        KDL::Frame current_pose;

        // RobotStatePublisher(robot_tree);
        // ros::NodeHandle n;
        // std::string joint_name;

        // if (!n.getParam("joint", joint_name)){
        //     ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
        //     return false;
        // }
        // ROS_INFO("name_space: ,'%s'", n.getNamespace().c_str());
        // ROS_INFO("joint_name: ,'%s'", joint_name.c_str());
        // joint_urdf = urdf_model.getJoint(joint_name);
        // if (!joint_urdf){
        //     ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        //     status = false;
        // }
        return status; // TODO CHECK status * exit_value too
    }
    
    int Controller::loadTree()
    {
        if (!this->urdf_model_.initFile(this->urdf_file_)){
            ROS_ERROR("Failed to parse urdf file");
            return 1;
        }
        
        if (!kdl_parser::treeFromUrdfModel(this->urdf_model_, this->robot_tree_)){
            ROS_ERROR("Failed to construct kdl tree");
            return 1;
        }

        return 0;
    }
    void Controller::setLowCmd(unitree_legged_msgs::LowCmd next_LowCmd)
    {
        this->next_LowCmd_ = next_LowCmd;
    }
    unitree_legged_msgs::LowCmd Controller::getLowCmd()
    {
        return next_LowCmd_;
    }
                /* FUNCTIONS FROM  BODY.CPP */

    // TODO TUNING
    void Controller::initMotorParams()
    {
        for(int i=0; i<4; i++)
        {
            // UNDER LOWCMD 
            next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
            next_LowCmd_.motorCmd[i*3+0].Kp = 70;
            next_LowCmd_.motorCmd[i*3+0].dq = 0;
            next_LowCmd_.motorCmd[i*3+0].Kd = 3;
            next_LowCmd_.motorCmd[i*3+0].tau = 0;
            next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
            next_LowCmd_.motorCmd[i*3+1].Kp = 180;
            next_LowCmd_.motorCmd[i*3+1].dq = 0;
            next_LowCmd_.motorCmd[i*3+1].Kd = 8;
            next_LowCmd_.motorCmd[i*3+1].tau = 0;
            next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
            next_LowCmd_.motorCmd[i*3+2].Kp = 300;
            next_LowCmd_.motorCmd[i*3+2].dq = 0;
            next_LowCmd_.motorCmd[i*3+2].Kd = 15;
            next_LowCmd_.motorCmd[i*3+2].tau = 0;
        }
        for(int i=0; i<robot_->num_joints; i++){
            next_LowCmd_.motorCmd[i].q = robot_->low_state_.motorState[i].q;
        }
        cmh_->gazeboSendLowCmd(next_LowCmd_);
    }

    void Controller::standUp()
    {   
        double pos[robot_->num_joints] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                        0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
        moveDesiredQs(pos, 2*1000);
    }
    void Controller::moveDesiredQs(double* desiredQs, double duration)
    {
        double lastPos[this->robot_->num_joints], percent;

        // for(int j=0; j<this->robot_->num_joints; j++) lastPos[j] = robot_->low_state_.motorState[j].q;
        for( int i=0; i < duration ; i++)
        {
            percent = (double)i/duration;
            for(int j=0; j < this->robot_->num_joints; j++)
            {
                next_LowCmd_.motorCmd[j].q = robot_->low_state_.motorState[j].q * (1-percent) + desiredQs[j]*percent;
            } 
            // TODO NOTIFY SEND THE MOTOR COMMAND
            /* send nextMotorCmd through lowCmd*/ 
            cmh_->gazeboSendLowCmd(this->next_LowCmd_);

        }
        
    }
}