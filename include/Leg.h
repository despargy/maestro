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

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#ifndef _LEG_H_
#define _LEG_H_

namespace RCD
{
    class Leg
    {
    public:

        int id;
        int n_superV_joints;
        std::string base_frame;
        std::string tip_frame;
        double prob_stab, w0;
        //KDL
        KDL::Chain kdl_chain;
        KDL::Jacobian jacobian_kdl;
        KDL::JntArray q, q_out;    // Joint pos qs
        KDL::Frame p_frame;          // Tip pose with respect to CoM 
        boost::scoped_ptr<KDL::ChainFkSolverPos> kdl_solver_pos;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> kdl_solver;
        //Eigen
        Eigen::MatrixXd J;            // Jacobian Eigen
        Eigen::Affine3d p, p_out;
        Eigen::Vector3d f, f_cmd, tau; //applyied force to the tip
        Eigen::Vector3d f_tf_toBase;

        Eigen::Matrix4d g_o;
        Eigen::Matrix4f g_o_world;

        // init W once
        Eigen::Vector3d wv_leg;
        Leg();
        ~Leg();

        void initLegs(int i, std::string name, KDL::Tree robot_kin);
        void kdlSolver();
        void IKkdlSolver(Eigen::Matrix4d M);

    
    };



}

#endif