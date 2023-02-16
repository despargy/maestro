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
        // Pass Tree form URDF
        this->loadTree();
        // pas num of joints
        robot_->num_joints = robot_kin.getNrOfJoints();  
        // time
        this->maestro_time = 0.0;
    }
    Controller::~Controller()
    {
        std::cout<<"Controller De-Constructor"<<std::endl;
    }

    void Controller::initControl()
    {
        // Init Leg manager for each leg
        this->initLegsControl();
        // Motor Params
        this->initMotorParams();
        // from robot state to joint array
        this->getJointQs(); // stored at leg 'q'
        for(int l = 0 ; l < n_leg ; l++)
            this->leg_mng[l].kdl_solver_pos->JntToCart(leg_mng[l].q,leg_mng[l].p);
        
        // set gains
        // TODO
    }
    void Controller::initLegsControl()
    {
        // 4 Leg controller
        this->n_leg = 4;
        leg_mng = new Leg[n_leg];
        std::string l_name[n_leg] = {"FR_foot","FL_foot","RR_foot","RL_foot"};
        for(int i = 0; i < n_leg ; i++)
            leg_mng[i].initLegs(i, l_name[i], robot_kin);
        if (n_leg*leg_mng[0].n_superV_joints != robot_->num_joints)  //eq. joint distribution, 3 per leg 
        ROS_ERROR("Robot Joints Number Not Matching");
    }
    void Controller::getJointQs()
    {
        for(int i = 0 ; i <  n_leg ; i++)
        {
            // as the num of n_superV_joints
            leg_mng[i].q(0) = this->robot_->low_state_.motorState[i*3 + 0].q;
            leg_mng[i].q(1) = this->robot_->low_state_.motorState[i*3 + 1].q;
            leg_mng[i].q(2) = this->robot_->low_state_.motorState[i*3 + 2].q;
        }
    }
    void Controller::loop()
    {   

        while(true) //
        {
            update();
            // for each leg find the jacobian, stored at leg 'jacobian_kdl'
            std::cout << "R=" << std::endl << this->robot_->com_q.toRotationMatrix() << std::endl;
            // std::cout << "R=" << std::endl << this->robot_->com_q.toRotationMatrix() << std::endl;
            // calc next Cmd
            // send Cmd
        }
    }
    void Controller::update()
    {   
        // dt compute and update
        this->maestro_time += 0.1; //TODO
        // get joint pos (?and VEL?) from MotorState.q to JntArray
        this->getJointQs(); // stored at leg 'q'
        // solve Jacs etc. for each leg
        // robot state updates automatically
        for(int l = 0; l < n_leg ; l++)
            leg_mng[l].reSolver();
        
        // Compute the forward kinematics and Jacobian

    }
    void Controller::loadTree()
    {
        // read URDF from parameter server and parse it into a KDL::Tree
        if (!kdl_parser::treeFromParam("/robot_description", this->robot_kin))
        throw std::runtime_error("Could not find robot URDF in parameter '/robot_description'.");
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
        if(!cmh_->real_experiment_)
        {
            for(int i=0; i<4; i++)
            {
                // Init motor Parameter for Gazebo
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
            for(int i=0; i<robot_->num_joints; i++)
            {
                next_LowCmd_.motorCmd[i].q = robot_->low_state_.motorState[i].q;
            }
        }
        else
        {   
            ROS_INFO("SOSOSOSOSOSOSSOSO tune");   //TODO 
            // for(int i=0; i<4; i++)
            // {
            //     // Init motor Parameter for Real
            //     next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
            //     next_LowCmd_.motorCmd[i*3+0].Kp = 70;
            //     next_LowCmd_.motorCmd[i*3+0].dq = 0;
            //     next_LowCmd_.motorCmd[i*3+0].Kd = 3;
            //     next_LowCmd_.motorCmd[i*3+0].tau = 0;
            //     next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
            //     next_LowCmd_.motorCmd[i*3+1].Kp = 180;
            //     next_LowCmd_.motorCmd[i*3+1].dq = 0;
            //     next_LowCmd_.motorCmd[i*3+1].Kd = 8;
            //     next_LowCmd_.motorCmd[i*3+1].tau = 0;
            //     next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
            //     next_LowCmd_.motorCmd[i*3+2].Kp = 300;
            //     next_LowCmd_.motorCmd[i*3+2].dq = 0;
            //     next_LowCmd_.motorCmd[i*3+2].Kd = 15;
            //     next_LowCmd_.motorCmd[i*3+2].tau = 0;
            // }
            // for(int i=0; i<robot_->num_joints; i++)
            // {
            //     next_LowCmd_.motorCmd[i].q = robot_->low_state_.motorState[i].q;
            // }
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
        std::cout<< robot_->low_state_ <<std::endl;
        double pos[robot_->num_joints] ,lastPos[robot_->num_joints], percent;
        for(int j=0; j<12; j++) lastPos[j] = robot_->low_state_.motorState[j].q;
        for(int i=1; i<=duration; i++)
        {
            if(!ros::ok()) break;
            percent = (double)i/duration;
            for(int j=0; j<robot_->num_joints; j++){
                next_LowCmd_.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
                ROS_INFO("q =  %f", next_LowCmd_.motorCmd[j].q);
            }
            /* send nextMotorCmd through lowCmd*/ 
            cmh_->sendLowCmd(this->next_LowCmd_);
            // ros::spinOnce();
            // ros::Duration(0.5).sleep(); // sleep for 0.5 seconds
            // sleep(3);
      
        }

    }

}
