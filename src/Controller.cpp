#include <Controller.h>

namespace RCD
{
                /* CONSTRUCTOR - DE  */

    Controller::Controller()
    {
        ROS_INFO("Controller Constructor");
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
        // Controller Gains
        this->kp = -500.0;
        this->ko = -1.0;
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
        this->initMotorParamsHard();
        // get joint qs and foot forces
        this->getLegQF(); // stored at leg 'q'
        for(int l = 0 ; l < this->n_leg ; l++)
            this->leg_mng[l].kdl_solver_pos->JntToCart(leg_mng[l].q,leg_mng[l].p_frame);
    }
    void Controller::initLegsControl()
    {
        // 4 Leg controller
        this->n_leg = 4;
        leg_mng = new Leg[this->n_leg];
        std::string l_name[this->n_leg] = {"FR_foot","FL_foot","RR_foot","RL_foot"};
        for(int i = 0; i < this->n_leg ; i++)
            leg_mng[i].initLegs(i, l_name[i], robot_kin);
        if (this->n_leg*leg_mng[0].n_superV_joints != robot_->num_joints)  //eq. joint distribution, 3 per leg 
        ROS_ERROR("Robot Joints Number Not Matching");
    }
    Eigen::Matrix3d Controller::scewSymmetric(Eigen::Vector3d t)
    {
        Eigen::Matrix3d t_hat;
        t_hat << 0, -t(2), t(1),
            t(2), 0, -t(0),
            -t(1), t(0), 0;
        return t_hat;
    }
    void Controller::getLegQF()
    {
        // robot state updates automatically
        for(int i = 0 ; i <  this->n_leg ; i++)
        {
            // get Joints
            leg_mng[i].q(0) = this->robot_->low_state_.motorState[i*3 + 0].q;
            leg_mng[i].q(1) = this->robot_->low_state_.motorState[i*3 + 1].q;
            leg_mng[i].q(2) = this->robot_->low_state_.motorState[i*3 + 2].q;
            // get foot Force on tip  
            leg_mng[i].f(0) = this->robot_->low_state_.eeForce[leg_mng[i].id].x;
            leg_mng[i].f(1) = this->robot_->low_state_.eeForce[leg_mng[i].id].y;
            leg_mng[i].f(2) = this->robot_->low_state_.eeForce[leg_mng[i].id].z;          
        }
    }
    void Controller::solveJacP()
    {
        // robot state updates automatically
        for(int l = 0; l < this->n_leg ; l++)
            // calc Jacobian and Pos
            leg_mng[l].kdlSolver();

    }
    void Controller::computeSudoGq()
    {
        // compute Gq
        // top Identities remain the same
        for(int l = 0; l < this->n_leg ; l++)
            this->robot_->Gq.block(3,l*3,3,3) =  this->scewSymmetric(this->robot_->R_c*this->leg_mng[l].p.translation()); //eq. 2 //SCEW   

        this->robot_->Gq_sudo = this->robot_->W_inv * this->robot_->Gq.transpose()*(this->robot_->Gq*this->robot_->W_inv*this->robot_->Gq.transpose()).inverse() ;
        // Eigen::VectorXd fv;
        // fv.resize(6);
        // fv << 0,0,1,0,0,0;
        
        // std::cout<<"Gq_sudo \n"<<-this->robot_->Gq_sudo*fv<<std::endl;
    
    }
    void Controller::setMotorModeGains()
    {
        if(!cmh_->real_experiment_)
        {
            for(int i=0; i<this->n_leg; i++)
            {
                // Init motor Parameter for Gazebo
                this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+0].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+0].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+0].Kd = 3;
                this->next_LowCmd_.motorCmd[i*3+0].tau = 0;
                this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+1].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+1].Kd = 8;
                this->next_LowCmd_.motorCmd[i*3+1].tau = 0;
                this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+2].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+2].Kd = 15;
                this->next_LowCmd_.motorCmd[i*3+2].tau = 0;
            }
        }
        else
        {
            //SOSOSOSSOSO TODO ASK what about tau 0, is ok due to previous q?
            for(int i=0; i<this->n_leg; i++)
            {
                // Init motor Parameter for Gazebo
                this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+0].Kp = 5.0;
                this->next_LowCmd_.motorCmd[i*3+0].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+0].Kd = 1.0;
                this->next_LowCmd_.motorCmd[i*3+0].tau = 0;
                this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+1].Kp = 5.0;
                this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+1].Kd = 1.0;
                this->next_LowCmd_.motorCmd[i*3+1].tau = 0;
                this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+2].Kp = 5.0;
                this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+2].Kd = 1.0;
                this->next_LowCmd_.motorCmd[i*3+2].tau = 0;
            }
        }
    }
    void Controller::loop()
    {   
        // init control time counter
        this->time_start = std::chrono::system_clock::now();

        Eigen::Vector3d p_d;
        Eigen::Vector3d p_d0(this->robot_->p_c);  // init pd0 from current state

        Eigen::Quaterniond stand_up_Q(0.99999, -0.00016,-0.00135, -0.00188);
        stand_up_Q.normalize();
        // init pd0 from current orientatio or replace to set your desired oriantetion
        Eigen::Matrix3d R_d = this->robot_->R_c;//stand_up_Q.toRotationMatrix();

        Eigen::Vector3d e_p, e_o;
        Eigen::AngleAxisd ang;
        Eigen::Matrix3d Re;

        Eigen::VectorXd fcontrol; // vector to help with eq. 11
        fcontrol.resize(6);

        // this->setMotorModeGains();
        for(int i=0; i<this->n_leg; i++)
        {
            // Init motor Parameter for Gazebo
            this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
            this->next_LowCmd_.motorCmd[i*3+0].Kp = 0;
            this->next_LowCmd_.motorCmd[i*3+0].dq = 0;
            this->next_LowCmd_.motorCmd[i*3+0].Kd = 3;
            this->next_LowCmd_.motorCmd[i*3+0].tau = 0;
            this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
            this->next_LowCmd_.motorCmd[i*3+1].Kp = 0;
            this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
            this->next_LowCmd_.motorCmd[i*3+1].Kd = 8;
            this->next_LowCmd_.motorCmd[i*3+1].tau = 0;
            this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
            this->next_LowCmd_.motorCmd[i*3+2].Kp = 0;
            this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
            this->next_LowCmd_.motorCmd[i*3+2].Kd = 15;
            this->next_LowCmd_.motorCmd[i*3+2].tau = 0;
        }
        while(true) //
        {
            this->time_elapsed =  std::chrono::system_clock::now() - this->time_start;
            p_d(0) = p_d0(0) + 0.1*sin(2*M_PI*0.1*this->time_elapsed.count()); 
            p_d(1) = p_d0(1) + 0.1*cos(2*M_PI*0.1*this->time_elapsed.count()); 
            p_d(2) = p_d0(2);

            std::cout << "pd0 \n"<<p_d0 << std::endl;
            std::cout << "robot pc \n"<<this->robot_->p_c << std::endl;

            // dt compute and update
            // compute the updated variables/Matrix
            this->update();
            // compute Fc based on error
            e_p = this->robot_->p_c - p_d;
            Re = this->robot_->R_c*R_d.transpose();
            ang.fromRotationMatrix(Re);
            e_o = ang.angle()*ang.axis();

            fcontrol.block(0,0,3,1) = this->kp*e_p;
            fcontrol.block(3,0,3,1) = this->ko*e_o;

            this->robot_->F_c = fcontrol + this->robot_->gc;  // eq.11 TODO
            this->robot_->F_a = this->robot_->Gq_sudo*this->robot_->F_c ; 
            for(int l = 0; l < this->n_leg ; l++)
            {
                this->leg_mng[l].f_cmd = -this->robot_->F_a.block(l*3,0,3,1);
                leg_mng[l].tau = (this->robot_->R_c*(leg_mng[l].J.block<3,3>(0,0))).transpose()*leg_mng[l].f_cmd;
            }
            this->setNewCmd();
            sleep(0.01); // TODO adapt it
        }
    }
    void Controller::setNewCmd()
    {
        for(int l = 0; l < this->n_leg ; l++)
        {
            next_LowCmd_.motorCmd[l*3+0].tau = leg_mng[l].tau(0);
            next_LowCmd_.motorCmd[l*3+1].tau = leg_mng[l].tau(1);
            next_LowCmd_.motorCmd[l*3+2].tau = leg_mng[l].tau(2);
        }
        cmh_->sendLowCmd(this->next_LowCmd_);
    }
    void Controller::update()
    {   
        // get current leg info Joint Qs and footForces
        this->getLegQF(); 
        // solve Jacs etc. for each leg
        this->solveJacP();
        // compute sudo Gq
        this->computeSudoGq();
    }
    void Controller::loadTree()
    {
        // read URDF from parameter server and parse it into a KDL::Tree
        if (!kdl_parser::treeFromParam("/robot_description", this->robot_kin))
        throw std::runtime_error("Could not find robot URDF in parameter '/robot_description'.");
    }
                /* FUNCTIONS FROM  BODY.CPP */
    void Controller::initMotorParamsHard()
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
            ROS_INFO("SOSOSOSOSOSOSSOSO tune");   //TODO ASK pos control has PosStopF VelStopF
            // TODO head? 
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
