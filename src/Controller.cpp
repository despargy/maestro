#include <Controller.h>

namespace RCD
{
                /* CONSTRUCTOR - DE  */

    Controller::Controller()
    {
        ROS_INFO("Controller Constructor");
    }
    Controller::Controller(Robot* robot, CommunicationHandler* cmh, DataHandler* data_handler)
    {
        ROS_INFO("Controller Constructor");
        this->cmh_ = cmh ;
        this->robot_ = robot;
        this->data_handler_ = data_handler;
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
        if (!this->cmh_->nh_main_->getParam( this->ns + "/ki", this->ki)){
            ROS_ERROR("No ki given in namespace: '%s')", this->cmh_->nh_main_->getNamespace().c_str());
        }
        if (!this->cmh_->nh_main_->getParam( this->ns + "/alpha", this->alpha)){
            ROS_ERROR("No alpha given in namespace: '%s')", this->cmh_->nh_main_->getNamespace().c_str());
        }
        if (!this->cmh_->nh_main_->getParam( this->ns + "/main_path", this->data_handler_->main_path)){
            ROS_ERROR("No main_path given in namespace: '%s')", this->cmh_->nh_main_->getNamespace().c_str());
        }
        
        // Pass Tree form URDF
        this->loadTree();
        // pas num of joints
        robot_->num_joints = 12;//robot_kin.getNrOfJoints();   // CHANGED AFTER EXTRA FOOT ADDED TO SIMULATED IMU
        // for orientation tracking
        this->b_coef = 0.1;
        this->e_v.resize(6);

        /* swing leg control */
        t_swing = 3.0;

        vp_order = new int[n_leg];
        vp_order[0] = 0; vp_order[1] = 2; vp_order[2] = 3; vp_order[3] = 1; // define polygon order

        free_gait = new int[n_leg];
        free_gait[0] = 0; free_gait[1] = 3; free_gait[2] = 1; free_gait[3] = 2; // define gait order

    }
    Controller::~Controller()
    {
        std::cout<<"Controller De-Constructor"<<std::endl;
    }

    void Controller::initControl()
    {
        // Init Leg manager for each leg
        this->initLegsControl();

        // std::cout<<"Now you have low state data"<<std::endl;
        std::cout<<"BEFORE STARTING POSE"<<std::endl;

        // Set the starting pose before stands up
        this->startingPose(); 
        // get joint qs and solveKDL for initialization
        this->updateLegs();
        // update CoM state
        this->updateCoM();

        // set pointers to DataHandler
        this->initDataHandler();
        // init vvvv
        for(int l = 0; l < this->n_leg ; l++)
        {
            // update vvvv vector of robot                          // z stays 1.0 do not change
            this->robot_->vvvv.block(l*3,0,3,1) = this->leg_mng[l].wv_leg;   
        }
        // save as matrix the inverse of diagonal vvvv vector
        this->robot_->W_inv = (this->robot_->vvvv.asDiagonal()).inverse();
    }
    void Controller::initDataHandler()
    {
                        /* Robot var. to be logged */
        // p_c pointer 
        this->data_handler_->log_data.p_c = &(this->robot_->p_c);              
        // Weight vector
        this->data_handler_->log_data.vvvv = &(this->robot_->vvvv);     
        // p_d pointer 
        this->data_handler_->log_data.p_d = &(this->p_d);   
        this->data_handler_->log_data.p_T = &(this->math_lib.p_T);   

        // probs
        this->data_handler_->log_data.leg_prob_0 = &(this->leg_mng[0].prob_stab)  ;       
        this->data_handler_->log_data.leg_prob_1 = &(this->leg_mng[1].prob_stab)  ;       
        this->data_handler_->log_data.leg_prob_2 = &(this->leg_mng[2].prob_stab)  ;       
        this->data_handler_->log_data.leg_prob_3 = &(this->leg_mng[3].prob_stab)  ;       

                        /* Controller var. to be logged */
        // e_p pointer 
        this->data_handler_->log_data.e_p = &(this->e_p); 
        // e_o pointer 
        this->data_handler_->log_data.e_o = &(this->e_o); 
        // e_v pointer 
        this->data_handler_->log_data.e_v = &(this->e_v); 
        // t_real pointer 
        this->data_handler_->log_data.t_real = &(this->t_real); 
        // tv pointer 
        this->data_handler_->log_data.tv = &(this->tv); 
        // d_tv pointer 
        this->data_handler_->log_data.d_tv = &(this->d_tv);       

        // x leg world
        this->data_handler_->log_data.leg_0_x = &(this->leg_mng[0].g_o_world(0,3))  ;       
        this->data_handler_->log_data.leg_1_x = &(this->leg_mng[1].g_o_world(0,3))  ;       
        this->data_handler_->log_data.leg_2_x = &(this->leg_mng[2].g_o_world(0,3))  ;       
        this->data_handler_->log_data.leg_3_x = &(this->leg_mng[3].g_o_world(0,3))  ;   

        // y leg world
        this->data_handler_->log_data.leg_0_y = &(this->leg_mng[0].g_o_world(1,3))  ;       
        this->data_handler_->log_data.leg_1_y = &(this->leg_mng[1].g_o_world(1,3))  ;       
        this->data_handler_->log_data.leg_2_y = &(this->leg_mng[2].g_o_world(1,3))  ;       
        this->data_handler_->log_data.leg_3_y = &(this->leg_mng[3].g_o_world(1,3))  ; 

        // z leg world
        this->data_handler_->log_data.leg_0_z = &(this->leg_mng[0].g_o_world(2,3))  ;       
        this->data_handler_->log_data.leg_1_z = &(this->leg_mng[1].g_o_world(2,3))  ;       
        this->data_handler_->log_data.leg_2_z = &(this->leg_mng[2].g_o_world(2,3))  ;       
        this->data_handler_->log_data.leg_3_z = &(this->leg_mng[3].g_o_world(2,3))  ; 

        // locomotion swing
        this->data_handler_->log_data.d_traj_tip_x = &(this->d_traj_tipframe(0))  ;       
        this->data_handler_->log_data.d_traj_tip_y = &(this->d_traj_tipframe(1))  ;       
        this->data_handler_->log_data.d_traj_tip_z = &(this->d_traj_tipframe(2))  ;       

        // desired pose for locomotion swing 
        this->data_handler_->log_data.d_traj_Oframe_x = &(this->d_traj_0frame(0))  ;       
        this->data_handler_->log_data.d_traj_Oframe_y = &(this->d_traj_0frame(1))  ;       
        this->data_handler_->log_data.d_traj_Oframe_z = &(this->d_traj_0frame(2))  ; 


        // locomotion swing 0 TODO change id
        this->data_handler_->log_data.swing_now_x = &(this->leg_mng[0].p.translation()(0))  ;       
        this->data_handler_->log_data.swing_now_y = &(this->leg_mng[0].p.translation()(1))  ;       
        this->data_handler_->log_data.swing_now_z = &(this->leg_mng[0].p.translation()(2))  ;  

        // locomotion swing 0 TODO change id
        this->data_handler_->log_data.q_out_0 = &(this->leg_mng[0].q_out(0));              
        this->data_handler_->log_data.q_out_1 = &(this->leg_mng[0].q_out(1));              
        this->data_handler_->log_data.q_out_2 = &(this->leg_mng[0].q_out(2));              

    }
    void Controller::initLegsControl()
    {
        // 4 Leg controller
        this->n_leg = 4;
        leg_mng = new Leg[this->n_leg];
        std::string l_name[this->n_leg] = {"FR_foot","FL_foot","RR_foot","RL_foot"};
        for(int i = 0; i < this->n_leg ; i++)
            leg_mng[i].initLegs(i, l_name[i], robot_kin);

        // if (this->n_leg*leg_mng[0].n_superV_joints != robot_->num_joints)  //eq. joint distribution, 3 per leg this cannot be used becasue we add extra foot link for fake imu data
        // ROS_ERROR("Robot Joints Number Not Matching");
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
        {
            // calc Jacobian and Pos
            // need for eq. 4
            leg_mng[l].kdlSolver();
            this->robot_->LegR_frame[l] = leg_mng[l].p.matrix().block(0,0,3,3);
        }
    }
    void Controller::computeWeights()
    {

        for(int l = 0; l < this->n_leg ; l++)
        {
            this->leg_mng[l].prob_stab = std::fmin(this->cmh_->slip[l],1.0)/1.0;
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
    void Controller::setMaestroMotorGains()
    {
        if(!cmh_->real_experiment_)
        {
            for(int i=0; i<this->n_leg; i++)
            {
                // Init motor Parameter for Gazebo
                this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+0].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+0].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+0].Kd = 1.5;
                this->next_LowCmd_.motorCmd[i*3+0].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+1].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+1].Kd = 1.5;
                this->next_LowCmd_.motorCmd[i*3+1].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+2].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+2].Kd = 1.5;
                this->next_LowCmd_.motorCmd[i*3+2].tau = 0.0f;
            }
        }
        else
        {
            for(int i=0; i<this->n_leg; i++)
            {
                // Init motor Parameter for Real Robot
                this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+0].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+0].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+0].Kd = 1.0;
                this->next_LowCmd_.motorCmd[i*3+0].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+1].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+1].Kd = 1.0;
                this->next_LowCmd_.motorCmd[i*3+1].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+2].Kp = 0;
                this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+2].Kd = 1.0;
                this->next_LowCmd_.motorCmd[i*3+2].tau = 0.0f;
            }
        }
    }
    void Controller::startingPose()
    {
        // For real robot to get the frst low state data
        if(this->cmh_->real_experiment_)
        {
            std::cout<<"IS REAL firstCommandForRealRobot"<<std::endl;
            this->firstCommandForRealRobot();
        }
        // Motor Params
        this->initMotorParamsHard();

        // gravity compensation change tau // SIMULATION ONLY
        ROS_INFO("gravComp(): starts");
        this->gravComp(); // runs for 5 secs
        ROS_INFO("gravComp(): ends");

        this->cmh_->sendLowCmd(this->next_LowCmd_); // LALA den xreiazetai genika
        ros::Duration(0.002).sleep();
        
        double start_pos[robot_->num_joints] = {0.2,  +1.5, -M_PI, 0.2,  +1.5, -M_PI, 
                                            0.2,  +1.5, -M_PI,0.2,  +1.5, -M_PI};
        moveDesiredQs(start_pos, 2*1000);


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
        initLocomotion();
        this->p_d = this->math_lib.get_pDesiredTrajectory(p_d0, 0.0);
        // Var for Real GO1 CoM 
        Eigen::Vector3d com_p_prev;
        Eigen::Matrix3d R_CoM_prev, dR_CoM;
        com_p_prev = p_d0;
        R_CoM_prev = R_d_0;

        setMaestroMotorGains();
        ros::Duration sleep_dt_ROS = ros::Duration(this->dt);


        while(this->robot_->KEEP_CONTROL & ros::ok()) 
        {

            // updates Legs variables n' Jacobian Matrix
            this->updateLegs();

            if (this->cmh_->INF)
            {
                // compute Weights based on prob for slip detection
                this->computeWeightsInfinity(); 
            }
            else if (this->cmh_->SLIP_DETECTION)
            {
                // compute Weights based on prob for slip detection
                this->computeWeights(); 
            }
            // give dt or keep old time to compute ros dt?
            this->t_real += this->dt;
            this->t_to_use = this->t_real;

            if(this->cmh_->ADAPT_B)
            {
                this->computeBeta_t(); // updates d_tv in Controller
                this->tv += this->d_tv*this->dt; 
                this->t_to_use = this->tv;
            }

            getTrajD();

            // update CoM state
            this->updateCoM();
            // updat CoM velocity
            if(cmh_->real_experiment_)
            {
                // compute CoM velocity
                robot_->dp_c = this->math_lib.get_dp_CoM(com_p_prev, this->robot_->p_c, this->dt);  
                dR_CoM = this->math_lib.get_dR_CoM(R_CoM_prev, this->robot_->R_c, this->dt); 
                robot_->w_c = this->math_lib.scewSymmetricInverse(dR_CoM*this->robot_->R_c.transpose());
                
                // update
                com_p_prev = this->robot_->p_c;
                R_CoM_prev = this->robot_->R_c;
            }
            else
            {
                this->updateVelocityCoM(); // TODO uncomment this, remove the next lines
            }
            
            this->positionError();
            this->velocityError();
            this->updateControlLaw(robot_->w_c);    // updates Coriolis/Inertia Matrix etc.
            this->fComputations();

            // Torque control per leg 
            for(int l = 0; l < this->n_leg ; l++)
            {
                this->leg_mng[l].f_cmd = -this->robot_->F_a.block(l*3,0,3,1); // slip Fa eq. 3
                leg_mng[l].tau =  (this->robot_->R_c*(leg_mng[l].J.block<3,3>(0,0))).transpose()*leg_mng[l].f_cmd; // compute eq. 4
            }

            // FOOT_IMU_ID IS FOOT to publish rotation
            int FOOT_IMU_ID = 0;
            this->cmh_->publishRotation(this->robot_->R_c*this->leg_mng[FOOT_IMU_ID].p.matrix().block(0,0,3,3));
            
            updateCoMTipsWorld();

            // Write a new line at csv
            this->cmh_->TIPS ? this->data_handler_->logDataTips() : this->data_handler_->logData();

            // send New Torque Command
            this->cmh_->INF & t_to_use>3 ? this->setNewCmdRise() : this->setNewCmd();
            sleep_dt_ROS.sleep();
            
        }
    }
    void Controller::fComputations()
    {
        // first term of Fc eq. 11
        fcontrol1.block(0,0,3,1) = ddp_d;
        fcontrol1.block(3,0,3,1) = this->math_lib.deriv_RcRdTwd( RcRdTwd, this->robot_->R_c*R_d.transpose()*w_d, this->dt); 
        RcRdTwd = this->robot_->R_c*R_d.transpose()*w_d;
        // second term of Fc eq. 11
        fcontrol2.block(0,0,3,1) = dp_d;
        fcontrol2.block(3,0,3,1) = RcRdTwd;
        // third term of Fc eq. 11
        fcontrol3.block(0,0,3,1) = -this->kp*this->e_p;
        fcontrol3.block(3,0,3,1) = -this->ko*this->e_o; 

        Gbc.block(3,0,3,3) = this->math_lib.scewSymmetric(this->robot_->R_c*pbc);

        // Final Fc ep. 11
        this->robot_->F_c = this->robot_->H_c*fcontrol1 + this->robot_->C_c*fcontrol2  + fcontrol3 - this->kv*this->e_v + Gbc*this->robot_->gc ;            // solve eq. 1 with respect to Fa

        this->robot_->F_a = this->robot_->Gq_sudo*this->robot_->F_c ;
    }
    void Controller::getTrajD()
    {
        // tv affects only desired trajectory scaling 
        // get next DESIRED position
        ddp_d = this->math_lib.get_ddpDesiredTrajectory(p_d0, p_d, dp_d, this->dt, t_to_use);
        dp_d = this->math_lib.get_dpDesiredTrajectory(p_d0, p_d, this->dt, t_to_use);
        p_d = this->math_lib.get_pDesiredTrajectory(p_d0, t_to_use);
            
        // get next DESIRED orientation
        dR_d = this->math_lib.get_dRDesiredRotationMatrix(Q_0, R_d, this->dt, t_to_use);
        R_d = this->math_lib.get_RDesiredRotationMatrix(Q_0, t_to_use);
        // DESIRED angular velocity of Com
        w_d = this->math_lib.scewSymmetricInverse(dR_d*R_d.transpose());
            
    }
    void Controller::positionError()
    {
        // compute position ERROR
        this->e_p = this->robot_->p_c - p_d;
        // compute orientation ERROR
        Re = this->robot_->R_c*R_d.transpose();
        ang.fromRotationMatrix(Re);
        this->e_o = ang.angle()*ang.axis();
    }
    void Controller::velocityError()
    {
        // compute velocity ERROR
        this->e_v.block(0,0,3,1) = robot_->dp_c - dp_d;
        this->e_v.block(3,0,3,1) = robot_->w_c - this->robot_->R_c*R_d.transpose()*w_d ;
        this->e_v.block(3,0,3,1) = 0.7*this->e_v.block(3,0,3,1) ;

    }
    void Controller::computeBeta_t()
    {
        this->d_tv = this->leg_mng[0].w0 / std::fmin( std::fmin( this->leg_mng[0].wv_leg(0), this->leg_mng[1].wv_leg(0)) , std::fmin( this->leg_mng[2].wv_leg(0),this->leg_mng[3].wv_leg(0)  ));
        std::cout<<"wwww  "<< this->robot_->vvvv.transpose() << " ------- "<<"d_tv  "<< d_tv<<std::endl;

    }
    void Controller::updateControlLaw(Eigen::Vector3d w_com)
    {
        // compute sudo Gq
        this->computeSudoGq();
        // update Coriolis and Inertia
        this->robot_->I_c = this->robot_->R_c*this->robot_->I*this->robot_->R_c.transpose();
        this->robot_->H_c.block(3,3,3,3) =  this->robot_->I_c ; 
        this->robot_->C_c.block(3,3,3,3) = this->math_lib.scewSymmetric(this->robot_->I_c*w_com);
    }
    void Controller::forceTrasform()
    {
        for(int l = 0 ; l < n_leg ; l++)
        {
            this->leg_mng[l].f_tf_toBase = this->robot_->R_c*this->leg_mng[l].p.matrix().block(0,0,3,3)*this->leg_mng[l].f;
        }
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
    }
    void Controller::setNewCmdRise()
    {
        for(int l = 0; l < this->n_leg ; l++)
        {
            next_LowCmd_.motorCmd[l*3+0].tau = (float) leg_mng[l].tau(0);
            next_LowCmd_.motorCmd[l*3+1].tau = (float) leg_mng[l].tau(1);
            next_LowCmd_.motorCmd[l*3+2].tau = (float) leg_mng[l].tau(2);
        }
        
        double percent = (t_to_use-t_swing)/t_swing > 1 ? 1: (t_to_use-t_swing)/t_swing ;
        for(int j=robot_->swingL_id*3; j<robot_->swingL_id*3 + 3; j++){
            next_LowCmd_.motorCmd[j].q = q_start_swing[j]*(1-percent) + q_target_swing[j]*percent; 
            // ROS_INFO("q =  %f", next_LowCmd_.motorCmd[j].q);
        }

        if(cmh_->real_experiment_)
        {
            this->next_LowCmd_.motorCmd[robot_->swingL_id*3+0].Kp = 10;
            this->next_LowCmd_.motorCmd[robot_->swingL_id*3+1].Kp = 50;
            this->next_LowCmd_.motorCmd[robot_->swingL_id*3+2].Kp = 70;

        }
        else
        {
            this->next_LowCmd_.motorCmd[robot_->swingL_id*3+0].Kp = 70;
            this->next_LowCmd_.motorCmd[robot_->swingL_id*3+1].Kp = 180;
            this->next_LowCmd_.motorCmd[robot_->swingL_id*3+2].Kp = 300;           
        }

        cmh_->sendLowCmd(this->next_LowCmd_);
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

            for(int i=0; i<n_leg; i++)
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
            for(int i=0; i<n_leg; i++)
            {
                // Init motor Parameter for Real
                this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+0].Kp = 10.0;
                this->next_LowCmd_.motorCmd[i*3+0].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+0].Kd = 1.5;
                this->next_LowCmd_.motorCmd[i*3+0].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+1].Kp = 50.0;
                this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+1].Kd = 1.5;
                this->next_LowCmd_.motorCmd[i*3+1].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+2].Kp = 70.0;
                this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+2].Kd = 1.5;
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
        double pos[robot_->num_joints] = {0.0, 0.67, -1.5, -0.0, 0.67, -1.5, 
                                            0.0, 0.67, -1.5, -0.0, 0.67, -1.5};
        moveDesiredQs(pos, 2*1000);

    }
    void Controller::moveDesiredQs(double* targetPos, double duration)
    {
        double pos[robot_->num_joints] ,lastPos[robot_->num_joints], percent;
        for(int j=0; j<12; j++)
        {
            lastPos[j] = robot_->low_state_.motorState[j].q;
        }   
        for(int i=1; i<=duration; i++)
        {
            if(!ros::ok()) break;
            percent = (double)i/duration;
            for(int j=0; j<robot_->num_joints; j++){
                next_LowCmd_.motorCmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
                // ROS_INFO("q =  %f", next_LowCmd_.motorCmd[j].q);
            }
            /* send nextMotorCmd through lowCmd*/ 
            cmh_->sendLowCmd(this->next_LowCmd_);
            ros::Duration(0.002).sleep();
        }

    }
    void Controller::firstCommandForRealRobot()
    {
        next_LowCmd_.head[0] = 0xFE;
        next_LowCmd_.head[1] = 0xEF;
        next_LowCmd_.levelFlag = 0xff; // LOWLEVEL
        for(int i=0; i<n_leg; i++)
        {
            // Init motor Parameter for Real
            this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
            this->next_LowCmd_.motorCmd[i*3+0].Kp = 0.0;
            this->next_LowCmd_.motorCmd[i*3+0].dq = 0;
            this->next_LowCmd_.motorCmd[i*3+0].Kd = 0.0;
            this->next_LowCmd_.motorCmd[i*3+0].tau = 0.0f;
            this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
            this->next_LowCmd_.motorCmd[i*3+1].Kp = 0.0;
            this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
            this->next_LowCmd_.motorCmd[i*3+1].Kd = 0.0;
            this->next_LowCmd_.motorCmd[i*3+1].tau = 0.0f;
            this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
            this->next_LowCmd_.motorCmd[i*3+2].Kp = 0.0;
            this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
            this->next_LowCmd_.motorCmd[i*3+2].Kd = 0.0;
            this->next_LowCmd_.motorCmd[i*3+2].tau = 0.0f;
        }
        // notice until now any low stat command is recieved
        // do not set q, only tau with gains ->0
        // gravity compensation TODO increase for real robot
        this->next_LowCmd_.motorCmd[0].tau = -0.0f; //FR_0
        this->next_LowCmd_.motorCmd[3].tau = +0.0f; //FL_0
        this->next_LowCmd_.motorCmd[6].tau = -0.0f;  //RR_0
        this->next_LowCmd_.motorCmd[9].tau = +0.0f; //RL_0
        int motiontime = 0;
        while(motiontime < 1500)
        {
            /* send nextMotorCmd through lowCmd*/ 
            cmh_->sendLowCmd(this->next_LowCmd_);
            ros::Duration(0.002).sleep();

            motiontime++;
        }
    }
    void Controller::updateCoM()
    {
        this->robot_->p_c = this->robot_->p_c_update;
        this->robot_->R_c = this->robot_->R_c_update;
    }
    void Controller::updateVelocityCoM()
    {
        this->robot_->dp_c = this->robot_->dp_c_update;
        this->robot_->w_c = this->robot_->w_c_update;
    }
    void Controller::computeWeightsInfinity()
    {
        
        for(int l = 0; l < this->n_leg ; l++)
        {

            this->leg_mng[l].prob_stab = std::fmin(this->cmh_->slip[l],1.0)/1.0;
            this->leg_mng[l].wv_leg(1) = this->alpha*(1.0 - this->leg_mng[l].prob_stab)*dt + this->leg_mng[l].wv_leg(1) ; // y
            this->leg_mng[l].wv_leg(0) = this->alpha*(1.0 - this->leg_mng[l].prob_stab)*dt + this->leg_mng[l].wv_leg(0); // x
            
            /* Added to simuate swing leg weights t inf */
            if( l == robot_->swingL_id)
                
                if (t_to_use>t_swing-dt)
                    this->leg_mng[l].wv_leg = this->leg_mng[0].w0*Eigen::Vector3d::Ones() + std::numeric_limits<double>::infinity()*Eigen::Vector3d::Ones(); 
                else
                    this->leg_mng[l].wv_leg = this->leg_mng[0].w0*Eigen::Vector3d::Ones() + atanh(t_to_use/t_swing)*Eigen::Vector3d::Ones(); 
            
            // update vvvv vector of robot                          // z stays 1.0 do not change
            this->robot_->vvvv.block(l*3,0,3,1) = this->leg_mng[l].wv_leg;   
        }
        // save as matrix the inverse of diagonal vvvv vector
        this->robot_->W_inv = (this->robot_->vvvv.asDiagonal()).inverse();
    }
    void Controller::computeWeightsSwing()
    {
        
        for(int l = 0; l < this->n_leg ; l++)
        {
            this->leg_mng[l].prob_stab = std::fmin(this->cmh_->slip[l],1.0)/1.0;
            this->leg_mng[l].wv_leg(1) = this->alpha*(1.0 - this->leg_mng[l].prob_stab)*dt + this->leg_mng[l].wv_leg(1) ; // y
            this->leg_mng[l].wv_leg(0) = this->alpha*(1.0 - this->leg_mng[l].prob_stab)*dt + this->leg_mng[l].wv_leg(0); // x
            
            /* Added to simuate swing leg weights t inf */
            if( l == robot_->swingL_id)
                this->leg_mng[l].wv_leg = this->leg_mng[0].w0*Eigen::Vector3d::Ones() + 3500*math_lib.superGaussian(A,b,t_half_swing-t0_superG,t_phase - t_half_swing)*Eigen::Vector3d::Ones(); 
            
            // update vvvv vector of robot                          // z stays 1.0 do not change
            this->robot_->vvvv.block(l*3,0,3,1) = this->leg_mng[l].wv_leg;   
        }
        // save as matrix the inverse of diagonal vvvv vector
        this->robot_->W_inv = (this->robot_->vvvv.asDiagonal()).inverse();
        // std::cout<<"this"<< this->robot_->W_inv<<std::endl;
    }
    void Controller::initTarget()
    {
        this->t_phase = 0.0; //t_to_use - t0;
        this->t0_phase = t_to_use; // when that phase started

        std::vector<std::pair<double, double> > vp;
        for(int l=0; l<this->n_leg; l++)
        {
            // each tips' x,y {world} position
            vp.push_back({this->leg_mng[vp_order[l]].g_o_world(0,3),this->leg_mng[vp_order[l]].g_o_world(1,3)}); 
        }
            

        std::pair<double, double> C = math_lib.find_Centroid(vp);
        Eigen::Vector3d target(C.first,C.second,0.4); //robot_->p_c(2)

        // set target goal position target, starting position, rotation target
        math_lib.updateTarget(target, robot_->p_c, robot_->R_c); 

        e_p_int = Eigen::Vector3d::Zero();
        e_o_int = Eigen::Vector3d::Zero();
        pid_out = Eigen::VectorXd::Zero(6);

    }
    void Controller::inverseTip()
    {

        float r1 = 0.008, r2=0.025;     

        if(t_phase<t0_swing)
        {
            d_tip_pos<< 0.0, 0.0, 0.0, 1.0; 
            d_tip_vel<< 0.0, 0.0, 0.0;

        }
        else if( t_phase>=t0_swing & t_phase<=(t0_swing + 1/freq_swing) ) 
        {
            // std::cout<<"THIS--------------------------------------------"<< std::endl;

            d_tip_pos << r1*(2*M_PI*freq_swing*(t_phase-t0_swing)-sin(2*M_PI*freq_swing*(t_phase-t0_swing))) , leg_mng[(int)robot_->swingL_id].y_tip,  r2*(1-cos(2*M_PI*freq_swing*(t_phase-t0_swing))) , 1.0;
            d_tip_vel << r1*(2*M_PI*freq_swing - 2*M_PI*freq_swing*cos(2*M_PI*freq_swing*(t_phase-t0_swing))), 0.0,  r2*2*M_PI*freq_swing*sin(2*M_PI*freq_swing*(t_phase-t0_swing));

        }
        else 
        {
            d_tip_vel<< 0.0, 0.0, 0.0;

        }  
        
        d_traj_tipframe(0) = d_tip_pos(0);
        d_traj_tipframe(1) = d_tip_pos(1);
        d_traj_tipframe(2) = d_tip_pos(2);

        Eigen::Matrix4f BC_T = Eigen::Matrix4f::Zero(); // let B be system B at tip, then BO_T is the transformation from B to world frame {0}, with:
        BC_T(3,3) = 1.0;
        BC_T.block(0,0,3,3) = Eigen::Matrix3f::Identity() ;// robot_->g_com_init.block(0,0,3,3).cast<float>();// ->g_com_init.block(0,0,3,3).inverse().cast<float>(); // B from C is inv. the robot R_c orientation as the robot frame {0}
        BC_T.block(0,3,3,1) = leg_mng[(int)robot_->swingL_id].g_0bo_init.block(0,3,3,1).cast<float>();  // translation as tip init pose, from {0}
        
        // desired swinging-tip trajectory represented from {0} is:
        Eigen::Vector4f d_CoM_pos =  BC_T*d_tip_pos;
        d_traj_0frame = d_CoM_pos.block(0,0,3,1); // cut last 1
        // d_vel_0frame = BC_T.block(0,0,3,3)*d_tip_vel;

        CLIK(d_traj_0frame, d_tip_vel);

        
    }
    void Controller::CLIK(Eigen::Vector3f pd_0frame_, Eigen::Vector3f dpd_0frame_)
    {

        Eigen::Vector3f d_q_ = (   robot_->R_c* leg_mng[(int)robot_->swingL_id].J.block<3,3>(0,0)).inverse().cast<float>()*(dpd_0frame_ - 8*(leg_mng[(int)robot_->swingL_id].g_o_world.block(0,3,3,1).cast<float>() - pd_0frame_) );
        // std::cout<<"leg_mng[(int)robot_->swingL_id].g_o_world.block(0,3,3,1).cast<float>():"<< leg_mng[(int)robot_->swingL_id].g_o_world.block(0,3,3,1).cast<float>().transpose()<<std::endl;
        // std::cout<<"pd_0frame_:"<< pd_0frame_.transpose()<<std::endl;
        // std::cout<<"p--------------------------------------------"<< std::endl;


        leg_mng[(int)robot_->swingL_id].q_out(0) = d_q_(0)*dt + leg_mng[(int)robot_->swingL_id].q_out(0);
        leg_mng[(int)robot_->swingL_id].q_out(1) = d_q_(1)*dt + leg_mng[(int)robot_->swingL_id].q_out(1);
        leg_mng[(int)robot_->swingL_id].q_out(2) = d_q_(2)*dt + leg_mng[(int)robot_->swingL_id].q_out(2);


        leg_mng[(int)robot_->swingL_id].dq_out(0) = d_q_(0);
        leg_mng[(int)robot_->swingL_id].dq_out(1) = d_q_(1);
        leg_mng[(int)robot_->swingL_id].dq_out(2) = d_q_(2);



    }
    void Controller::setNewCmdSwing_Kp()
    {
        for(int l = 0; l < this->n_leg ; l++)
        {
            next_LowCmd_.motorCmd[l*3+0].tau = (float) leg_mng[l].tau(0);
            next_LowCmd_.motorCmd[l*3+1].tau = (float) leg_mng[l].tau(1);
            next_LowCmd_.motorCmd[l*3+2].tau = (float) leg_mng[l].tau(2);
        }
        for(int j=0; j<3; j++)
        {
            next_LowCmd_.motorCmd[(int)robot_->swingL_id*3+j].q = (float) leg_mng[(int)(robot_->swingL_id)].q_out(j); 
        }
        
        cmh_->sendLowCmd(this->next_LowCmd_);
    }
    void Controller::setNewCmdSwing()
    {
        for(int l = 0; l < this->n_leg ; l++)
        {
            next_LowCmd_.motorCmd[l*3+0].tau = (float) leg_mng[l].tau(0);
            next_LowCmd_.motorCmd[l*3+1].tau = (float) leg_mng[l].tau(1);
            next_LowCmd_.motorCmd[l*3+2].tau = (float) leg_mng[l].tau(2);

        }
        
        cmh_->sendLowCmd(this->next_LowCmd_);
    }
    void Controller::setPhaseTarget()
    {
        this->t_phase = 0.0; //t_to_use - t0;
        this->t0_phase = t_to_use; // when that phase started
        

        std::vector<std::pair<double, double> > vp;
        for(int l=0; l<this->n_leg; l++)
        {
            // each tips' x,y {world} position, NOT consider swing leg
            if((vp_order[l] != robot_->swingL_id) )
            {
                vp.push_back({this->leg_mng[vp_order[l]].g_o_world(0,3),this->leg_mng[vp_order[l]].g_o_world(1,3)}); 
            }
            
        }
            
        std::pair<double, double> C = math_lib.find_Centroid(vp);
        Eigen::Vector3d target(C.first,C.second,robot_->z);


        // set target goal position target, starting position, rotation target
        math_lib.updateTarget(target, robot_->p_c, robot_->R_c); 

        e_p_int = Eigen::Vector3d::Zero();
        e_o_int = Eigen::Vector3d::Zero();
        pid_out = Eigen::VectorXd::Zero(6);


        leg_mng[(int)robot_->swingL_id].g_0bo_init  = leg_mng[(int)robot_->swingL_id].g_o_world.cast <float> ();
        // g_0bo_init  = leg_mng[(int)robot_->swingL_id].g_o.cast <float> ();
        // robot_->g_com_init = this->robot_->g_com;
        // leg_mng[(int)robot_->swingL_id].wv_leg = leg_mng[(int)robot_->swingL_id].w0*Eigen::Vector3d::Ones();  

        for(int l = 0; l < this->n_leg ; l++)
        {
            // update vvvv vector of robot                          // z stays 1.0 do not change
            leg_mng[l].wv_leg = leg_mng[l].w0*Eigen::Vector3d::Ones();  

            this->robot_->vvvv.block(l*3,0,3,1) = this->leg_mng[l].wv_leg;   
        }
        // save as matrix the inverse of diagonal vvvv vector
        this->robot_->W_inv = (this->robot_->vvvv.asDiagonal()).inverse();




    }
    void Controller::initLocomotion()
    {
        // set dt    
        this->dt = 0.002;
        this->t_real = -dt;  // in loop will be +dt = 0.0
        this->t_phase = -dt; //        -//-
        this->t0_phase = -dt;//          -//-
        this->tv = 0.0;
        this->d_tv = 1.0;
        this->t_to_use = -dt;//0.0; // will take values from t_real or virtual time tv
        
        this->A = 1.0;
        this->b = 10.0;
        this->t0_superG = 1.0;

        this->freq_swing = 1.0;// 0.5 // 2 sec duration, starts at:  t0_swing(0.5s) up to 1/freq_swing+t0_swing(2.5s)
        this->t0_swing = 1.2; //0.6
        this->t_half_swing = 1.7;//(1/freq_swing)/2 + t0_swing ; 
        this->swing_t_slot =  2.5; 
        
        this->updateCoM();

        // Desired position variables
        p_d0 = this->robot_->p_c;
        dp_d = Eigen::Vector3d::Zero();
        ddp_d = Eigen::Vector3d::Zero();

        R_d_0 =  this->robot_->R_c; 
        Q_0 = Eigen::Quaterniond(R_d_0); 
        Gbc = Eigen::MatrixXd::Identity(6,6);
        RcRdTwd = Eigen::Vector3d::Zero();
        fcontrol1.resize(6);
        fcontrol2.resize(6);
        fcontrol3.resize(6);
        pid_out.resize(6);

        pbc << this->robot_->pbc_x ,this->robot_->pbc_y , this->robot_->pbc_z ; // center of mass offset 
        robot_->z = 0.35;

        LOC_STATE = PH_TARGET;
        
        g_d = Eigen::Matrix4d::Zero();
        g_d(3,3) = 1;


        leg_mng[0].y_tip = leg_mng[0].g_o_world(1,3) - 0.01;
        leg_mng[1].y_tip = leg_mng[1].g_o_world(1,3) + 0.01;
        leg_mng[2].y_tip = leg_mng[2].g_o_world(1,3) - 0.01;
        leg_mng[3].y_tip = leg_mng[3].g_o_world(1,3) + 0.01;

    }
    void Controller::positionErrorTarget()
    {
        // compute position ERROR
        e_p = robot_->p_c - math_lib.p_T;
        // compute orientation ERROR
        Re = robot_->R_c*math_lib.R_T.transpose();
        ang.fromRotationMatrix(Re);
        e_o = ang.angle()*ang.axis();
    }
    void Controller::velocityErrorTarget()
    {
        // compute velocity ERROR
        this->e_v.block(0,0,3,1) = robot_->dp_c;
        this->e_v.block(3,0,3,1) = robot_->w_c ;
        this->e_v.block(3,0,3,1) = 0.7*this->e_v.block(3,0,3,1) ;
    }
    void Controller::PIDwithSat()
    {
        //PIDwithSAT
        // third term of Fc eq. 11
        pid_out.block(0,0,3,1) = -kp*e_p;
        pid_out.block(3,0,3,1) = -ko*e_o; 
        pid_out -=  this->kv*this->e_v ;
        // Add Integral -> Target Control
        for (int axis = 0; axis<3; axis++)
        {
            //position
            if(fabs(e_p(axis))<0.01)
            {
                e_p_int(axis) += e_p(axis)*dt;
            }
            //orientation
            if(fabs(e_o(axis))<0.2)
            {
                e_o_int(axis) += e_o(axis)*dt;
            }
        }
        pid_out.block(0,0,3,1) += -ki*e_p_int.block(0,0,3,1) ;
        pid_out.block(3,0,3,1) += -0.3*ki*e_o_int ;


        if(pid_out.block(0,0,3,1).norm()>80.0){
            pid_out.block(0,0,3,1) = pid_out.block(0,0,3,1) * 80.0 / pid_out.block(0,0,3,1).norm();
        }
        if(pid_out.block(3,0,3,1).norm()>80.0){
            pid_out.block(3,0,3,1) = pid_out.block(3,0,3,1) * 80.0 / pid_out.block(3,0,3,1).norm();
        }

        
        // std::cout<< "e_p_int "<<e_p_int.transpose() << std::endl;
        // std::cout<< "e_o_int "<<e_o_int.transpose() << std::endl;
        // std::cout<< "pid_out "<<pid_out.transpose() << std::endl;

        Gbc.block(3,0,3,3) = math_lib.scewSymmetric(robot_->R_c*pbc);
        pid_out += Gbc*robot_->gc;
    }
    void Controller::fComputationsTarget()
    {
        // Final Fc ep. 11 -> Target Control
        robot_->F_c = pid_out ;
        // solve eq. 1 with respect to Fa 
        robot_->F_a = robot_->Gq_sudo*robot_->F_c ;
        for(int l = 0; l < n_leg ; l++)
        {
            leg_mng[l].f_cmd = -robot_->F_a.block(l*3,0,3,1); // slip Fa eq. 3
            // saturation Fz
            leg_mng[l].f_cmd(2) =  std::fmin(leg_mng[l].f_cmd(2), 0.0);
            leg_mng[l].tau =  (robot_->R_c*(leg_mng[l].J.block<3,3>(0,0))).transpose()*leg_mng[l].f_cmd; // compute eq. 4
            // std::cout<<"J_"<<l<<":" << leg_mng[l].J.block<3,3>(0,0) << std::endl;

        }
        // std::cout<<"--------------------------------------" << std::endl;
    }
    void Controller::locomotion_loop()
    {

        initLocomotion();
        int loc_i = -1 ;
        while(this->robot_->KEEP_CONTROL & ros::ok())
        {
            ros::Time start_time = ros::Time::now();
            // ros::WallTime start_time = ros::WallTime::now();

            t_real += dt;
            t_to_use = t_real;
            // TODO, here add ADAPT_B if so {}
            t_phase = t_to_use - t0_phase;

            updateLegs();
            updateCoM();
            updateVelocityCoM(); // TODO change for real robot case
            updateCoMTipsWorld();

            if( t_phase > swing_t_slot) // THIS
            {
                LOC_STATE = PH_TARGET;
                std::cout<<"LOC_STATE = PH_TARGET"<<std::endl;
            }
                
            switch (LOC_STATE)
            {
            case PH_TARGET:

                // change swing leg by free gat order, circle using mod and counter
                robot_->swingL_id = free_gait[(int)(++loc_i%n_leg)];
                
                if(loc_i%n_leg == 0) //single locomotion cycle is done
                {
                    for(int l=0; l<n_leg; l++)
                    {
                        initQout(l); // set all q_out as zeros 
                        std::cout<<"changed qs"<<std::endl;
                    }

                }
                std::cout<<robot_->swingL_id<<std::endl;
                
                setPhaseTarget();
                // record data, change swing pointer of DataHandler
                this->data_handler_->log_data.swing_now_x = &(this->leg_mng[(int)robot_->swingL_id].p.translation()(0));       
                this->data_handler_->log_data.swing_now_y = &(this->leg_mng[(int)robot_->swingL_id].p.translation()(1));       
                this->data_handler_->log_data.swing_now_z = &(this->leg_mng[(int)robot_->swingL_id].p.translation()(2));  

                // record data, change swing pointer of DataHandler
                this->data_handler_->log_data.q_out_0 = &(this->leg_mng[(int)robot_->swingL_id].q_out(0));              
                this->data_handler_->log_data.q_out_1 = &(this->leg_mng[(int)robot_->swingL_id].q_out(1));              
                this->data_handler_->log_data.q_out_2 = &(this->leg_mng[(int)robot_->swingL_id].q_out(2));  
                
                LOC_STATE = PH_SWING; // change state

            case PH_SWING:

                computeWeightsSwing(); 
                positionErrorTarget();        // error in position
                velocityErrorTarget();        // error in velocity
                computeSudoGq();        // sudo Gq
                PIDwithSat();
                fComputationsTarget();
                inverseTip(); 

                if( t_phase<=( 2*t_half_swing - t0_superG ) ) //  t0_swing + 1/freq_swing
                {
                //     inverseTip(); 
                    // setMaestroMotorGainsWalk_Kp();
                    // setNewCmdSwing_Kp();

                    setReNewCmd_Kp();
                }                
                else
                {
                    // std::cout<<"in else"<<std::endl;
                    // setMaestroMotorGainsWalk();
                    // setNewCmdSwing(); 
                    setReNewCmd();
                }

            }

            // Log data - csv format 
            data_handler_->logDataWalk();

            // waitWithRosWallTime(dt);
            waitWithRosGazeboTime(dt, start_time);
            // setReNewCmd();

        }
    }

    void Controller::waitWithRosWallTime(double dt,  ros::WallTime start_time) 
    {        
        while ((ros::WallTime::now() - start_time).toSec() < dt) {
            // Wait until the specified time duration 'dt' has passed
            ros::spinOnce(); // Process any pending ROS messages
        }
        ros::spinOnce(); // Process any pending ROS messages
    }


    void Controller::waitWithRosGazeboTime(double dt, ros::Time start_time) 
    {
        
        while ((ros::Time::now() - start_time).toSec() < dt) {
            // Wait until the specified time duration 'dt' has passed
            ros::spinOnce(); // Process any pending ROS messages
        }
        ros::spinOnce(); // Process any pending ROS messages
    }



    void Controller::initQout(int l)
    {
        leg_mng[l].q_out(0) = leg_mng[l].q(0);
        leg_mng[l].q_out(1) = leg_mng[l].q(1);
        leg_mng[l].q_out(2) = leg_mng[l].q(2);
    }
    void Controller::updateCoMTipsWorld()
    {
        // tip world frame pos
        robot_->g_com.block(0,0,3,3) = robot_->R_c;
        robot_->g_com.block(0,3,3,1) = robot_->p_c;
        for(int l = 0; l < n_leg ; l++)
        {
            leg_mng[l].g_o_world = (robot_->g_com*leg_mng[l].g_o).cast <float> ();
        }
    }
    void Controller::setMaestroMotorGainsWalk()
    {
        if(!cmh_->real_experiment_)
        {
            for(int i=0; i<this->n_leg; i++)
            {
                // Init motor Parameter for Gazebo
                this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+0].Kp = 0.0;
                this->next_LowCmd_.motorCmd[i*3+0].dq = 0; 
                this->next_LowCmd_.motorCmd[i*3+0].Kd = 2.5; //2.5
                this->next_LowCmd_.motorCmd[i*3+0].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+1].Kp = 0.0;
                this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+1].Kd = 4.5; //4.5
                this->next_LowCmd_.motorCmd[i*3+1].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+2].Kp = 0.0;
                this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+2].Kd = 6.5; //6.5
                this->next_LowCmd_.motorCmd[i*3+2].tau = 0.0f;
            }

        }

    }
    void Controller::setMaestroMotorGainsWalk_Kp()
    {
        if(!cmh_->real_experiment_)
        {
            for(int i=0; i<this->n_leg; i++)
            {
                // Init motor Parameter for Gazebo
                this->next_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+0].Kp = 0.0;
                this->next_LowCmd_.motorCmd[i*3+0].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+0].Kd = 2.5; //2.5
                this->next_LowCmd_.motorCmd[i*3+0].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+1].Kp = 0.0;
                this->next_LowCmd_.motorCmd[i*3+1].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+1].Kd = 4.5; //4.5
                this->next_LowCmd_.motorCmd[i*3+1].tau = 0.0f;
                this->next_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                this->next_LowCmd_.motorCmd[i*3+2].Kp = 0.0;
                this->next_LowCmd_.motorCmd[i*3+2].dq = 0;
                this->next_LowCmd_.motorCmd[i*3+2].Kd = 6.5; //6.5
                this->next_LowCmd_.motorCmd[i*3+2].tau = 0.0f;
            }

            this->next_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+0].Kp = 3.0; //3.0
            this->next_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+1].Kp = 7.0; //5.0
            this->next_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+2].Kp = 9.0; //8.0

            this->next_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+0].Kd = 1.0;
            this->next_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+1].Kd = 1.0;
            this->next_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+2].Kd = 1.0;   //1.5
        }

    }

    void Controller::setReNewCmd_Kp()
    {
        unitree_legged_msgs::LowCmd renew_LowCmd_; 

        if(!cmh_->real_experiment_)
        {
            for(int i=0; i<this->n_leg; i++)
            {
                // Init motor Parameter for Gazebo
                renew_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                renew_LowCmd_.motorCmd[i*3+0].Kp = 0.0;
                renew_LowCmd_.motorCmd[i*3+0].dq = 0;
                renew_LowCmd_.motorCmd[i*3+0].Kd = 1.5; //2.5
                renew_LowCmd_.motorCmd[i*3+0].tau = (float) leg_mng[i].tau(0);
                renew_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                renew_LowCmd_.motorCmd[i*3+1].Kp = 0.0;
                renew_LowCmd_.motorCmd[i*3+1].dq = 0;
                renew_LowCmd_.motorCmd[i*3+1].Kd = 2.5; //4.5
                renew_LowCmd_.motorCmd[i*3+1].tau = (float) leg_mng[i].tau(1);
                renew_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                renew_LowCmd_.motorCmd[i*3+2].Kp = 0.0;
                renew_LowCmd_.motorCmd[i*3+2].dq = 0;
                renew_LowCmd_.motorCmd[i*3+2].Kd = 5.5; //6.5
                renew_LowCmd_.motorCmd[i*3+2].tau =  (float) leg_mng[i].tau(2);
            }

        }

        for(int j=0; j<3; j++)
        {
            renew_LowCmd_.motorCmd[(int)robot_->swingL_id*3+j].q = (float) leg_mng[(int)(robot_->swingL_id)].q_out(j); 
            renew_LowCmd_.motorCmd[(int)robot_->swingL_id*3+j].dq = (float) leg_mng[(int)(robot_->swingL_id)].dq_out(j); 

        }
        
        renew_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+0].Kp = 7.0; //3.0
        renew_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+1].Kp = 12.0; //5.0
        renew_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+2].Kp = 16.0; //8.0

        renew_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+0].Kd = 1.5;
        renew_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+1].Kd = 2.0;
        renew_LowCmd_.motorCmd[(int)(robot_->swingL_id)*3+2].Kd = 3.5;   //1.5


        cmh_->sendLowCmd(renew_LowCmd_);

    }

    void Controller::setReNewCmd()
    {
        unitree_legged_msgs::LowCmd renew_LowCmd_; 

        if(!cmh_->real_experiment_)
        {
            for(int i=0; i<this->n_leg; i++)
            {
                // Init motor Parameter for Gazebo
                renew_LowCmd_.motorCmd[i*3+0].mode = 0x0A;
                renew_LowCmd_.motorCmd[i*3+0].Kp = 0.0;
                renew_LowCmd_.motorCmd[i*3+0].dq = 0; 
                renew_LowCmd_.motorCmd[i*3+0].Kd = 1.5; //2.5
                renew_LowCmd_.motorCmd[i*3+0].tau = (float) leg_mng[i].tau(0);
                renew_LowCmd_.motorCmd[i*3+1].mode = 0x0A;
                renew_LowCmd_.motorCmd[i*3+1].Kp = 0.0;
                renew_LowCmd_.motorCmd[i*3+1].dq = 0;
                renew_LowCmd_.motorCmd[i*3+1].Kd = 2.5; //4.5
                renew_LowCmd_.motorCmd[i*3+1].tau =  (float) leg_mng[i].tau(1);
                renew_LowCmd_.motorCmd[i*3+2].mode = 0x0A;
                renew_LowCmd_.motorCmd[i*3+2].Kp = 0.0;
                renew_LowCmd_.motorCmd[i*3+2].dq = 0;
                renew_LowCmd_.motorCmd[i*3+2].Kd = 5.5; //6.5
                renew_LowCmd_.motorCmd[i*3+2].tau = (float) leg_mng[i].tau(2);
            }
            
            cmh_->sendLowCmd(renew_LowCmd_);

        }

    }


}
