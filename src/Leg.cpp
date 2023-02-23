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
        f_cmd.resize(3);
        tau.resize(3);
        // Leg Weights based on Slip Prob 
        prob_stab = 1.0; // init as stable contact
        wv_leg = Eigen::Vector3d::Ones();  
    }
    void Leg::kdlSolver()
    {
        if ( !(kdl_solver->JntToJac(q, jacobian_kdl) >=0 ))
            ROS_ERROR("One leg unexpected Jacobian solution leg, '%d'", id);
        kdl_solver_pos->JntToCart(q,p_frame);
        J = jacobian_kdl.data; // jac KDL to Eigen  
        tf::transformKDLToEigen(p_frame, p); // p to eigen Affine
        // std::cout<<"tip pos"<<id<<" \n"<<p_frame.M<<std::endl;
        // Eigen::Matrix3d frame_m(p_frame.M);
        // std::cout<<p<<std::endl;
        mat_from_frame = p.matrix().block(0,0,3,3);
        // mframe*p
        // std::cout<<"rotation Matrix pos \t"<<id<<" \n"<<mat_from_frame.block(0,0,3,3)<<std::endl;


    }


}