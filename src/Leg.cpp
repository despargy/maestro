#include <Leg.h>

namespace RCD
{
    Leg::Leg()
    {
        ROS_INFO("Leg Constructor");
    }
    Leg::~Leg()
    {
        ROS_INFO("Leg De-Constructor");
    }
    void Leg::initLegs(int i, std::string name, KDL::Tree robot_kin)
    {
        id = i;
        //KDL init
        base_frame = "base";
        tip_frame = name;
        if (!robot_kin.getChain(base_frame, tip_frame, kdl_chain)) 
        {
            ROS_ERROR("Could not initialize chain object");
        }
        n_superV_joints = kdl_chain.getNrOfJoints();
        kdl_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));
        kdl_solver_pos.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
        jacobian_kdl.resize(n_superV_joints);
        q.resize(n_superV_joints);  
        q_out.resize(n_superV_joints);  
        //Eigen init
        J.resize(6, n_superV_joints);
        f.resize(3);
        f_tf_toBase.resize(3);
        f_cmd.resize(3);
        tau.resize(3);
        // Leg Weights based on Slip Prob 
        prob_stab = 1.0; // init as stable contact
        w0=35.0;
        wv_leg = w0*Eigen::Vector3d::Ones();  

        g_o = Eigen::Matrix4d::Zero();
        g_o(3,3) = 1;
        g_o_world = Eigen::Matrix4f::Zero();
    }
    void Leg::kdlSolver()
    {
        if ( !(kdl_solver->JntToJac(q, jacobian_kdl) >=0 ))
            ROS_ERROR("One leg unexpected Jacobian solution leg, '%d'", id);
        kdl_solver_pos->JntToCart(q,p_frame);
        J = jacobian_kdl.data; // jac KDL to Eigen  
        tf::transformKDLToEigen(p_frame, p); // p to eigen Affine
        g_o.block(0,0,3,3) = p.rotation();
        g_o.block(0,3,3,1) = p.translation();

    }
    void Leg::IkKDL(Eigen::Vector3d pos_dest)
    {

        //Creation of the solvers:
        KDL::ChainFkSolverPos_recursive fksolver1(kdl_chain);//Forward position solver
        KDL::ChainIkSolverVel_pinv iksolver1v(kdl_chain);//Inverse velocity solver
        KDL::ChainIkSolverPos_NR iksolver1(kdl_chain,fksolver1,iksolver1v,1000,1e-4);//Maximum 100 iterations, stop at accuracy 1e-6
        KDL::JntArray q_init;
        q_init.resize(n_superV_joints);  
        //Set destination frame
        KDL::Frame F_dest(KDL::Vector(pos_dest(0),pos_dest(1),pos_dest(2)));
        int ret = iksolver1.CartToJnt(q_init,F_dest,q_out);
        if(ret>=0){

        }else{
            printf("%s \n","Error: could not calculate IK");
        }

    }
    void Leg::IKkdlSolver(Eigen::Matrix4d M)
    {

        //Creation of the solvers:
        KDL::ChainFkSolverPos_recursive fksolver1(kdl_chain);//Forward position solver
        KDL::ChainIkSolverVel_pinv iksolver1v(kdl_chain);//Inverse velocity solver
        KDL::ChainIkSolverPos_NR iksolver1(kdl_chain,fksolver1,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
        KDL::JntArray q_init;
        q_init.resize(n_superV_joints);  
        //Set destination frame
        KDL::Frame F_dest;
        Eigen::Affine3d b;
        b.matrix() = M;

        tf::transformEigenToKDL(b, F_dest);
        int ret = iksolver1.CartToJnt(q_init,F_dest,q_out);

        // std::cout<<"q_test"<<q_test(0)<<", "<<q_test(1)<<", "<<q_test(2)<<std::endl;
        std::cout<<"q_out"<<q_out(0)<<", "<<q_out(1)<<", "<<q_out(2)<<std::endl;

        // std::cout<<"g_o"<<g_o.block(0,3,3,1)<<std::endl;


        // Create solver based on kinematic chain
        KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(kdl_chain);
        // Create the frame that will contain the results
        KDL::Frame cartpos;    
        // Eigen::Affine3d p_test;

        // Calculate forward position kinematics
        bool kinematics_status;
        // kinematics_status = fksolver.JntToCart(q,cartpos);
        // tf::transformKDLToEigen(cartpos, p_test); // p to eigen Affine

        // if(kinematics_status>=0){
        //     std::cout << "cartpos_now"<<p_test.translation() <<std::endl;
        //     printf("%s \n","Succes, thanks KDL!");
        // }else{
        //     printf("%s \n","Error: could not calculate forward kinematics :(");
        // }
        kinematics_status = fksolver.JntToCart(q_out,cartpos);
        tf::transformKDLToEigen(cartpos, p_out); // p to eigen Affine

        if(kinematics_status>=0){
            std::cout << "cartpos_desired"<<p_out.translation() <<std::endl;
            printf("%s \n","Succes, thanks KDL!");
        }else{
            printf("%s \n","Error: could not calculate forward kinematics :(");
        }
    }

}