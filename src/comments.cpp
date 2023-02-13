
    // // test functions cmh update robot
    // unitree_legged_msgs::IMU imu0;
    // imu0.quaternion[0] = 2;
    // cmh->robot_->setImuBase(imu0);
    // std::cout<<"robot->getImuBase()"<<std::endl;
    // std::cout<<robot->getImuBase()<<std::endl;


    // maestro::Foot f0 ;
    // f0.footForce[3] = 555;
    // robot->setFoot(f0);
    // std::cout<<"robot->getFoot()"<<std::endl;
    // std::cout<<robot->getFoot()<<std::endl;

    // maestro::MotorState* ms0_p ;
    // ms0_p = new maestro::MotorState[2];
    // ms0_p[0].q = 3;
    // robot->setMotorState(ms0_p);
    // std::cout<<"robot->getMotorState()"<<std::endl;
    // maestro::MotorState* l = robot->getMotorState();
    // std::cout<<robot->getMotorState()<<std::endl;

    // maestro::MotorState ms0_list[20];
    // for(int i = 0 ; i < 20 ; i++)
    // {
    //     ms0_list[i].q = i;
    // }
    // robot->setMotorStateAr(ms0_list);
    // maestro::MotorState* ms2_list = robot->getMotorStateAr();
    // for(int i = 0 ; i < 20 ; i++)
    // {
    //     std::cout<<ms2_list[i].q<< std::endl;
    // }

    // maestro::MotorCmd mcmd0_list[20];
    // for(int i = 0 ; i < 20 ; i++)
    // {
    //     mcmd0_list[i].q = i+20;
    // }
    // robot->setMotorCmdAr(mcmd0_list);
    // maestro::MotorCmd* mcmd2_list = robot->getMotorCmdAr();
    // for(int i = 0 ; i < 20 ; i++)
    // {
    //     std::cout<<mcmd2_list[i].q<< std::endl;
    // }
    // int *a;
    // a = new int(5);
    // for(int i=0 ; i < 5; i++ )
    // {
    //     // a[i] = i;
    //     std::cout<< a[i] << std::endl;
    // }

        // RCD::Robot robot2;

CONTROLLER

//     int Controller::initController()
//     {
//         try
//         {
//             // read URDF from parameter server and parse it into a KDL::Tree
//             KDL::Tree robot_kin;
//             if (!kdl_parser::treeFromParam("/robot_description", robot_kin))
//             throw std::runtime_error("Could not find robot URDF in parameter '/robot_description'.");
//             ROS_INFO("Tree ok");
//             std::cout<<robot_kin.getNrOfSegments()<<std::endl;
//             KDL::Chain chain;
//             robot_kin.getChain("base", "RL_foot", chain);
// ////////////////////////////////////////////////////////////

//             jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain));
//             jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(chain));

//             // resizes the joint state vectors in non-realtime
//             jnt_pos_.resize(chain.getNrOfJoints());
//             jnt_effort_.resize(chain.getNrOfJoints());
//             jacobian_.resize(chain.getNrOfJoints());

//             // sets the desired pose
//             reference_pose_ = KDL::Frame(KDL::Rotation::RPY(0,0,0),
//                                         KDL::Vector(0.7, -0.2, 0));

            

////////////////////////////////////////////////////////////
            // KDL::JntArray joint_pos(chain.getNrOfJoints());
            // KDL::Frame cart_pos;
            // KDL::ChainFkSolverPos_recursive fk_solver(chain);
            // fk_solver.JntToCart(joint_pos, cart_pos);
            // for(int i = 0 ; i < 12; i++)
            // {
            //     std::cout<<cart_pos.M.Quaternion<<std::endl;
            // }
//////////////////////////////
            // KDL::ChainJntToJacSolver kdl_solver(chain);
            // KDL::Jacobian jacobian_kdl(12);
            // KDL::JntArray q_in(12);
            // for(int i = 0 ; i < 12 ; i ++)
            // {
            //     q_in(i) = robot_->low_state_.motorState[i].q;
            // }
            // if (kdl_solver.JntToJac(q_in,jacobian_kdl) >= 0)
            //     ROS_INFO("GREATER");
            // for (int i = 0 ; i < 12 ; i++)
            // {
            //     std::cout<<jacobian_kdl.getColumn(i)[0]<<std::endl;
            // }
            
//////////////////////////////

// https://cpp.hotexamples.com/examples/kdl/Tree/getChain/cpp-tree-getchain-method-examples.html
            // KDL::JntArray q(chain.getNrOfJoints());
            // KDL::JntArray q_dot(chain.getNrOfJoints());
            // KDL::JntArray q_dotdot(chain.getNrOfJoints());
            // KDL::JntArray torques(chain.getNrOfJoints());
            // KDL::Wrenches f_ext;
            // f_ext.resize(chain.getNrOfSegments());

            // printf("RNE dynamics values \n");

            // KDL::ChainIdSolver_RNE *rneDynamics = new ChainIdSolver_RNE(chain, -grav);
            
            // for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
            // {
            //     q(i) = robot_->low_state_.motorState[i].q;
            //     q_dot(i) = robot_->low_state_.motorState[i].dq;
            //     q_dotdot(i) = robot_->low_state_.motorState[i].ddq;
            //     printf("q, qdot %f, %f\n", q(i), q_dot(i));
            // }

            
            // rneDynamics->CartToJnt(q, q_dot, q_dotdot, f_ext, torques);

            // for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i)
            // {
            //     printf("index, q, qdot, torques %d, %f, %f, %f\n", i, q(i), q_dot(i), torques(i));
            // }
            // /////////////// or 
            // // https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/examples/chainiksolverpos_lma_demo.cpp
            // KDL::ChainIkSolverPos_LMA solver(chain);

        // }
        // catch (const std::exception& e)
        // {
        //     ROS_ERROR("%s", e.what());
        // }

        // int status = loadTree(); // state 0 means OK

        // KDL::Chain chain;
        // bool exit_value = robot_tree_.getChain("FR_calf_joint","FR_hip_joint",chain);
        // if (exit_value)
        // {
        //     std::cout<<"getChain TRUE"<<std::endl;
        // }
        // else
        // {
        //     std::cout<<"getChain FALSE "<<std::endl;
        // }

        // // std::cout<<chain<<std::endl;
        // boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
        // KDL::Frame current_pose;

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
    //     return 0; // TODO CHECK status * exit_value too
    // }