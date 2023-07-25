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


}