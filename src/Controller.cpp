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
        // Read urdf
        if (!this->cmh_->nh_main_->getParam( this->ns + "/urdf_file_path", this->urdf_file_)){
            ROS_ERROR("No is_simulation given in namespace: '%s')", this->cmh_->nh_main_->getNamespace().c_str());
        }
        // Gains
        if (!this->cmh_->nh_main_->getParam( this->ns + "/kp", this->kp)){
            ROS_ERROR("No kp given in namespace: '%s')", this->cmh_->nh_main_->getNamespace().c_str());
        }
        if (!this->cmh_->nh_main_->getParam( this->ns + "/ko", this->ko)){
            ROS_ERROR("No ko given in namespace: '%s')", this->cmh_->nh_main_->getNamespace().c_str());
        }
        if (!this->cmh_->nh_main_->getParam( this->ns + "/kv", this->kp)){
            ROS_ERROR("No kv given in namespace: '%s')", this->cmh_->nh_main_->getNamespace().c_str());
        }
        // Pass Tree form URDF
        this->loadTree();
        // pas num of joints
        robot_->num_joints = robot_kin.getNrOfJoints();  
        // for orientation tracking
        this->b_coef = 0.1;
        this->alpha = 1000.0;

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
        // Set the starting pose before stands up
        // this->startingPose(); TODO
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
            // need for eq. 4
            leg_mng[l].kdlSolver();

    }
    void Controller::computeWeights(double dt)
    {

        for(int l = 0; l < this->n_leg ; l++)
        {
            this->leg_mng[l].prob_stab = this->cmh_->slip[l];
            this->leg_mng[l].wv_leg(1) = this->alpha*(1.0 - this->leg_mng[l].prob_stab)*dt + this->leg_mng[l].wv_leg(1) ; // y
            this->leg_mng[l].wv_leg(0) = this->alpha*(1.0 - this->leg_mng[l].prob_stab)*dt + this->leg_mng[l].wv_leg(0); // x
            
            // update vvvv vector of robot                          // z stays 1.0 do not change
            this->robot_->vvvv.block(l*3,0,3,1) = this->leg_mng[l].wv_leg;   
        }

        // save as matrix the inverse of diagonal vvvv vector
        this->robot_->W_inv = (this->robot_->vvvv.asDiagonal()).inverse();
    }
    void Controller::computeSudoGq()
    {
        // compute Gq eq. 2
        // top Identities remain the same
        for(int l = 0; l < this->n_leg ; l++)
        {
            this->robot_->Gq.block(3,l*3,3,3) =  this->math_lib.scewSymmetric(this->robot_->R_c*this->leg_mng[l].p.translation()); //eq. 2 //SCEW
        }
        // compute Gp_sude eq. 7
        this->robot_->Gq_sudo = this->robot_->W_inv * this->robot_->Gq.transpose()*(this->robot_->Gq*this->robot_->W_inv*this->robot_->Gq.transpose()).inverse() ;
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
                this->next_LowCmd_.motorCmd[i*3+0].Kd = 3.0;
                this->next_LowCmd_.motorCmd[i*3+0].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+1].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+1].Kd = 3.0;
                this->next_LowCmd_.motorCmd[i*3+1].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+2].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+2].Kd = 3.0;
                this->next_LowCmd_.motorCmd[i*3+2].tau = 0.0f;
                // WHAT ABOUT qS ? ASK
            }
        }
        else
        {
            //SOSOSOSSOSO TODO ASK what about tau 0, is ok due to previous q?
            for(int i=0; i<this->n_leg; i++)
            {
                // Init motor Parameter for Real Robot
                this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+0].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+0].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+0].Kd = 3.0;
                this->next_LowCmd_.motorCmd[i*3+0].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+1].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+1].Kd = 8.0;
                this->next_LowCmd_.motorCmd[i*3+1].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+2].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+2].Kd = 15.0;
                this->next_LowCmd_.motorCmd[i*3+2].tau = 0.0f;
                // WHAT ABOUT qS ? ASK
            }
        }
    }
    void Controller::startingPose()
    {
        next_LowCmd_.head[0] = 0xFE;
        next_LowCmd_.head[1] = 0xEF;
        next_LowCmd_.levelFlag = 0xff; // LOWLEVEL
        for (int i = 0; i < this->robot_->num_joints; i++)
        {
            next_LowCmd_.motorCmd[i].mode = 0x0A; 
            next_LowCmd_.motorCmd[i].q = PosStopF; 
            next_LowCmd_.motorCmd[i].Kp = 0;
            next_LowCmd_.motorCmd[i].dq = VelStopF; 
            next_LowCmd_.motorCmd[i].Kd = 0;
            next_LowCmd_.motorCmd[i].tau = 0;
        }
        this->cmh_->sendLowCmd(this->next_LowCmd_);
        sleep(5);

        // gravity compensation change tau
        ROS_INFO("gravComp(): starts");
        this->gravComp(); // runs for 5 secs
        ROS_INFO("gravComp(): ends");

        for(int i =0 ; i < n_leg ; i++)
        {
            next_LowCmd_.motorCmd[i*3+2].q = -M_PI; 
            next_LowCmd_.motorCmd[i*3+2].Kp = 70.0;
            next_LowCmd_.motorCmd[i*3+2].Kd = 3.0;
            
            next_LowCmd_.motorCmd[i*3+0].q = 0.2;
            next_LowCmd_.motorCmd[i*3+0].dq = 0.0;
            next_LowCmd_.motorCmd[i*3+0].Kp = 70.0;
            next_LowCmd_.motorCmd[i*3+0].Kd = 3.0;
            
            next_LowCmd_.motorCmd[i*3+1].q = +1.5;
            next_LowCmd_.motorCmd[i*3+1].dq = 0.0;
            next_LowCmd_.motorCmd[i*3+1].Kp = 70.0;
            next_LowCmd_.motorCmd[i*3+1].Kd = 3.0;
        }
        this->cmh_->sendLowCmd(this->next_LowCmd_);
        sleep(0.01);

    }
    void Controller::gravComp()
    {
        // gravity compensation
        this->next_LowCmd_.motorCmd[0].tau = -0.65f; //FR_0
        this->next_LowCmd_.motorCmd[3].tau = +0.65f; //FL_0
        this->next_LowCmd_.motorCmd[6].tau = -0.65f;  //RR_0
        this->next_LowCmd_.motorCmd[9].tau = +0.65f; //RL_0
    }
    void Controller::loop()
    {   
        // init control time counter
        this->time_start = std::chrono::system_clock::now();
        double dt;
        // Desired position variables
        Eigen::Vector3d p_d, dp_d, ddp_d;
        Eigen::Vector3d p_d0(this->robot_->p_c);  // init pd0 from current state
        p_d = this->math_lib.get_pDesiredTrajectory(p_d0, 0.0);
        dp_d = Eigen::Vector3d::Zero();
        ddp_d = Eigen::Vector3d::Zero();
        // Desired orientation variables
        Eigen::Matrix3d R_d, dR_d, ddR_d;
        Eigen::Matrix3d R_d_0 =  this->robot_->R_c; 
        Eigen::Quaterniond Q_0(R_d_0); 
        // desired angular veocity
        Eigen::Vector3d w_d;
        Eigen::Vector3d RcRdTwd = Eigen::Vector3d::Zero();
        // Error variables
        Eigen::Vector3d e_p, e_o;
        Eigen::VectorXd e_v;
        e_v.resize(6);
        Eigen::AngleAxisd ang;
        Eigen::Matrix3d Re;
        // vector to help with eq. 11
        Eigen::VectorXd fcontrol1,fcontrol2,fcontrol3; 
        fcontrol1.resize(6);
        fcontrol2.resize(6);
        fcontrol3.resize(6);

        this->setMotorModeGains();

        while(this->robot_->KEEP_CONTROL) //
        {
            // if (this->cmh_->SLIP_DETECTION)
            //     std::cout<<"SLIP_DETECTION true"<<std::endl;
            // else
            //     std::cout<<"SLIP_DETECTION false"<<std::endl;

            // get delta t
            this->time_elapsed =  std::chrono::system_clock::now() - this->time_start;
            dt = this->time_elapsed.count();

            // get next DESIRED position
            ddp_d = this->math_lib.get_ddpDesiredTrajectory(p_d0, p_d, dp_d, dt);
            dp_d = this->math_lib.get_dpDesiredTrajectory(p_d0, p_d, dt);
            p_d = this->math_lib.get_pDesiredTrajectory(p_d0, dt);
            // get next DESIRED orientation
            dR_d = this->math_lib.get_dRDesiredRotationMatrix(Q_0, R_d, dt);
            R_d = this->math_lib.get_RDesiredRotationMatrix(Q_0, dt);
            // DESIRED angular velocity of Com
            w_d = this->math_lib.scewSymmetricInverse(dR_d*R_d.transpose());

            // compute position ERROR
            e_p = this->robot_->p_c - p_d;
            // compute orientation ERROR
            Re = this->robot_->R_c*R_d.transpose();
            ang.fromRotationMatrix(Re);
            e_o = ang.angle()*ang.axis();
            // compute velocity ERROR
            e_v.block(0,0,3,1) = this->robot_->com_vel_linear - dp_d;
            e_v.block(3,0,3,1) = this->robot_->com_vel_ang - this->robot_->R_c*R_d.transpose()*w_d ;

            // updates Legs variables n' Jacobian Matrix
            this->updateLegs();
            if (this->cmh_->SLIP_DETECTION)
            {
                // compute Weights based on prob for slip detection
                this->computeWeights(dt);
            }
            // updates Coriolis/Inertia Matrix etc.
            this->updateControlLaw();

            // first term of Fc eq. 11
            fcontrol1.block(0,0,3,1) = ddp_d;
            fcontrol1.block(3,0,3,1) = this->math_lib.deriv_RcRdTwd( RcRdTwd, this->robot_->R_c*R_d.transpose()*w_d, dt); 
            RcRdTwd = this->robot_->R_c*R_d.transpose()*w_d;
            // second term of Fc eq. 11
            fcontrol2.block(0,0,3,1) = dp_d;
            fcontrol2.block(3,0,3,1) = RcRdTwd;
            // third term of Fc eq. 11
            fcontrol3.block(0,0,3,1) = -this->kp*e_p;
            fcontrol3.block(3,0,3,1) = -this->ko*e_o;

            // Final Fc ep. 11
            this->robot_->F_c = this->robot_->H_c*fcontrol1 + this->robot_->C_c*fcontrol2 + fcontrol3 - this->kv*e_v + this->robot_->gc ; 
            // solve eq. 1 with respect to Fa
            this->robot_->F_a = this->robot_->Gq_sudo*this->robot_->F_c ;

            // Torque control per leg 
            for(int l = 0; l < this->n_leg ; l++)
            {
                this->leg_mng[l].f_cmd = -this->robot_->F_a.block(l*3,0,3,1); // slip Fa eq. 3
                leg_mng[l].tau =  (this->robot_->R_c*(leg_mng[l].J.block<3,3>(0,0))).transpose()*leg_mng[l].f_cmd; // compute eq. 4
                // if (leg_mng[l].tau(0) > 5.0 || leg_mng[l].tau(0) < -5.0) 
                //     std::cout<< leg_mng[l].tau <<std::endl;
                // if (leg_mng[l].tau(1) > 5.0 || leg_mng[l].tau(1) < -5.0) 
                //     std::cout<< leg_mng[l].tau <<std::endl;
                // if (leg_mng[l].tau(2) > 5.0 || leg_mng[l].tau(2) < -5.0) 
                //     std::cout<< leg_mng[l].tau <<std::endl;
            }
            // send New Torque Command
            this->setNewCmd();
        }
    }
    void Controller::updateControlLaw()
    {
        // compute sudo Gq
        this->computeSudoGq();
        // update Coriolis and Inertia
        this->robot_->I_c = this->robot_->R_c*this->robot_->I*this->robot_->R_c.transpose();
        this->robot_->H_c.block(3,3,3,3) =  this->robot_->I_c ; 
        this->robot_->C_c.block(3,3,3,3) = this->math_lib.scewSymmetric(this->robot_->I_c*this->robot_->com_vel_ang);
    }
    void Controller::setNewCmd()
    {
        for(int l = 0; l < this->n_leg ; l++)
        {
            next_LowCmd_.motorCmd[l*3+0].tau = (float) leg_mng[l].tau(0);
            next_LowCmd_.motorCmd[l*3+1].tau = (float) leg_mng[l].tau(1);
            next_LowCmd_.motorCmd[l*3+2].tau = (float) leg_mng[l].tau(2);
        }
        cmh_->sendLowCmd(this->next_LowCmd_);
        sleep(0.01); // TODO adapt it 
    }
    void Controller::updateLegs()
    {   
        // get current leg info Joint Qs and footForces
        this->getLegQF(); 
        // solve Jacs etc. for each leg
        this->solveJacP();
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
        next_LowCmd_.head[0] = 0xFE;
        next_LowCmd_.head[1] = 0xEF;
        next_LowCmd_.levelFlag = 0xff; // LOWLEVEL
        if(!cmh_->real_experiment_)
        {

            for(int i=0; i<4; i++)
            {
                // Init motor Parameter for Gazebo
                next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                next_LowCmd_.motorCmd[i*3+0].Kp = 70;
                next_LowCmd_.motorCmd[i*3+0].dq = 0;
                next_LowCmd_.motorCmd[i*3+0].Kd = 3;
                next_LowCmd_.motorCmd[i*3+0].tau = 0.0f;
                next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                next_LowCmd_.motorCmd[i*3+1].Kp = 180;
                next_LowCmd_.motorCmd[i*3+1].dq = 0;
                next_LowCmd_.motorCmd[i*3+1].Kd = 8;
                next_LowCmd_.motorCmd[i*3+1].tau = 0.0f;
                next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                next_LowCmd_.motorCmd[i*3+2].Kp = 300;
                next_LowCmd_.motorCmd[i*3+2].dq = 0;
                next_LowCmd_.motorCmd[i*3+2].Kd = 15;
                next_LowCmd_.motorCmd[i*3+2].tau = 0.0f;
            }
            for(int i=0; i<robot_->num_joints; i++)
            {
                next_LowCmd_.motorCmd[i].q = robot_->low_state_.motorState[i].q;
            }
        }
        else
        {   
            ROS_INFO("SOSOSOSOSOSOSSOSO tune");   //TODO ASK pos control has PosStopF VelStopF
            for(int i=0; i<4; i++)
            {
                // Init motor Parameter for Real
                this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+0].Kp = 5.0;
                this->next_LowCmd_.motorCmd[i*3+0].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+0].Kd = 1.0;
                this->next_LowCmd_.motorCmd[i*3+0].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+1].Kp = 5.0;
                this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+1].Kd = 1.0;
                this->next_LowCmd_.motorCmd[i*3+1].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+2].Kp = 5.0;
                this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+2].Kd = 1.0;
                this->next_LowCmd_.motorCmd[i*3+2].tau = 0.0f;
            }
            for(int i=0; i<robot_->num_joints; i++)
            {
                next_LowCmd_.motorCmd[i].q = robot_->low_state_.motorState[i].q;
            }
        }

    }
    void Controller::standUp()
    {   
        double pos[robot_->num_joints] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                                            0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
        moveDesiredQs(pos, 4*1000);

        // When stand up - HIGH level - real robot
        // double pos[robot_->num_joints] = {0.0, 0.81, -1.5, 0.0, 0.81, -1.5, 
        //                                    0.0, 0.81, -1.5, -0.0, 0.81, -1.5};
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
            sleep(0.01); // TODO up to sleep(0.002) ~ 500Hz, better test sleep (0.01)
      
        }

    }

}
