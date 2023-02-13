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
        // read URDF from parameter server and parse it into a KDL::Tree
        KDL::Tree robot_kin;
        if (!kdl_parser::treeFromParam("/robot_description", robot_kin))
        throw std::runtime_error("Could not find robot URDF in parameter '/robot_description'.");
        ROS_INFO("Tree ok");
        std::cout<<robot_kin.getNrOfSegments()<<std::endl;
        // KDL::TreeJntToJacSolver tree_jnt_to_jac_solver(robot_kin);
        // for (int i = 0 ; i < 12; i++)
        // {
        //     jnt_pos_(i)= this->robot_->getLowState().motorState[i].q;
        // }
        // tree_jnt_to_jac_solver.JntToJac(jnt_pos_, jacobian_, "RL_foot")  ;
        KDL::Chain kdl_chain;
        std::string base_frame("base");
        std::string tip_frame("RL_foot");
        if (!robot_kin.getChain(base_frame, tip_frame, kdl_chain)) 
        {
            ROS_ERROR("Could not initialize chain object");
        }
        int n_joint = kdl_chain.getNrOfJoints();

        KDL::ChainJntToJacSolver kdl_solver(kdl_chain);
        KDL::Jacobian jacobian_kdl(n_joint);
        KDL::JntArray q_in(n_joint);
        if (kdl_solver.JntToJac(q_in,jacobian_kdl) >=0 )
        {
            std::cout<<"Expected to be >=0"<<std::endl;
        }
        double pos[robot_->num_joints] = {0.0, 0.0, -0.0};
        for(int i = 0; i < n_joint;  i++)
            q_in(i) = pos[i];
        if (kdl_solver.JntToJac(q_in,jacobian_kdl) >=0 )
        {
            std::cout<<"Expected to be >=0"<<std::endl;
        }
        for(int i =0 ; i < n_joint; i++)
        {
            for(int j =0 ; j < n_joint; j++)
            {
                std::cout<<jacobian_kdl(i,j)<<std::endl;
            }
        }

        return 0;
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

    }

    void Controller::standUp()
    {   
        double pos[robot_->num_joints] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                                            0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
        moveDesiredQs(pos, 2*1000);
    }
    void Controller::moveDesiredQs(double* targetPos, double duration)
    {
        double pos[12] ,lastPos[12], percent;
        for(int j=0; j<12; j++) lastPos[j] = robot_->low_state_.motorState[j].q;
        for(int i=1; i<=duration; i++)
        {
            if(!ros::ok()) break;
            percent = (double)i/duration;
            for(int j=0; j<12; j++){
                next_LowCmd_.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
                ROS_INFO("q =  %f", next_LowCmd_.motorCmd[j].q);
            }
        
            /* send nextMotorCmd through lowCmd*/ 
            cmh_->gazeboSendLowCmd(this->next_LowCmd_);
            // ros::spinOnce();
            // ros::Duration(5).sleep(); // sleep for 0.5 seconds
            // sleep(3);
      
        }

    }
    // void Controller::next(double* targetPos)
    // {
    //     for(int j=0; j<12; j++)
    //     {
    //         this->next_LowCmd_.motorCmd[j].q =  this->robot_->low_state_.motorState[j].q + (targetPos[j] - this->robot_->low_state_.motorState[j].q)/100;
    //     }

    // }
    void Controller::loop()
    {
        // robot_->getLowState();
        // next(STAND_POS);
        // cmh_->gazeboSendLowCmd(this->next_LowCmd_);        
        ROS_INFO("IN LOOP");
    }

}
