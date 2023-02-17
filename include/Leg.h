/* Header file for Leg class under RCD*/

#include <string.h>
#include <iostream>
#include <urdf/model.h>
#include <ros/ros.h> 
#include <boost/scoped_ptr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <unsupported/Eigen/MatrixFunctions>

#ifndef _LEG_H_
#define _LEG_H_

#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)
namespace RCD
{
    class Leg
    {
        // enum
        // {
        //     Joints = 3
        // };
        // // http://library.isr.ist.utl.pt/docs/roswiki/pr2_mechanism(2f)Tutorials(2f)Coding(20)a(20)realtime(20)Cartesian(20)controller(20)with(20)Eigen.html
        // typedef Eigen::Matrix<double, Joints, 1>  JointVector;
        // typedef Eigen::Affine3d                CartPose;
        // typedef Eigen::Matrix<double, 3, 1>       Cart3Vector;
        // typedef Eigen::Matrix<double, 6, 1>       Cart6Vector;
        // typedef Eigen::Matrix<double, 6, Joints>  JacobianMatrix;
    public:
        // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        int id;
        int n_superV_joints;
        std::string base_frame;
        std::string tip_frame;
        //KDL
        KDL::Chain kdl_chain;
        KDL::Jacobian jacobian_kdl;
        KDL::JntArray q;    // Joint pos qs
        KDL::Frame p_frame;          // Tip pose with respect to CoM //ASK
        boost::scoped_ptr<KDL::ChainFkSolverPos> kdl_solver_pos;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> kdl_solver;
        //Eigen
        Eigen::MatrixXd J;            // Jacobian Eigen
        Eigen::Affine3d p;
        Eigen::Vector3d f,f_cmd, tau; //applyied force to the tip  
        // Eigen::MatrixXd xdot;
        Leg();
        ~Leg();

        void initLegs(int i, std::string name, KDL::Tree robot_kin);
        void kdlSolver();

    
    };



}

#endif