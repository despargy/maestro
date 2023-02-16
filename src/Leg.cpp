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
        // p = KDL::Frame(KDL::Rotation::RPY(0,0,0),
        //                                 KDL::Vector(0.0, 0.67, -1.3));
    }
    void Leg::reSolver()
    {
        if ( !(kdl_solver->JntToJac(q, jacobian_kdl) >=0 ))
            ROS_ERROR("One leg unexpected Jacobian solution leg, '%d'", id);
        kdl_solver_pos->JntToCart(q,p);
        J = jacobian_kdl.data; // jac KDL to Eigen  
        tf::transformKDLToEigen(p, x_aff); // p to eigen Affine
        // std::cout<<"tip pos \n"<<p<<std::endl;
        // std::cout<<"x_aff \n"<<x_aff<<std::endl;

        // tf::transformKDLToEigen(jacobian_kdl, J); //TODO
        // J(0) = jacobian_kdl.getColumn(0);
        // Eigen::Affine3d ret;
        // tf::transformKDLToEigen(jacobian_kdl, ret);
    }


}