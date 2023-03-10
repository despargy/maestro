#include <string.h>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Dense>

#ifndef _MATH_H_
#define _MATH_H_

namespace RCD
{
    class Math
    {
    public:
        Math();
        ~Math();
        Eigen::Matrix3d scewSymmetric(Eigen::Vector3d t);
        Eigen::Vector3d scewSymmetricInverse(Eigen::Matrix3d m);
        Eigen::Vector3d get_pDesiredTrajectory(Eigen::Vector3d p_d0_, double t_real);
        Eigen::Vector3d get_dpDesiredTrajectory(Eigen::Vector3d p_d0_,Eigen::Vector3d p_d_cur, double dt, double t_real);
        Eigen::Vector3d get_ddpDesiredTrajectory(Eigen::Vector3d p_d0_,Eigen::Vector3d p_d_cur,Eigen::Vector3d dp_d_cur, double dt, double t_real);
        Eigen::Matrix3d get_RDesiredRotationMatrix(Eigen::Quaterniond Q_0, double t_real);
        Eigen::Matrix3d get_dRDesiredRotationMatrix(Eigen::Quaterniond Q_0, Eigen::Matrix3d R_cur, double dt, double t_real);
        Eigen::Vector3d deriv_RcRdTwd(Eigen::Vector3d RcRdTwd_prev,Eigen::Vector3d RcRdTwd_cur, double dt);
        Eigen::Vector3d get_dp_CoM(Eigen::Vector3d com_p_prev,Eigen::Vector3d com_p_cur, double dt);
        Eigen::Matrix3d get_dR_CoM(Eigen::Matrix3d R_CoM_prev,Eigen::Matrix3d R_CoM_cur, double dt);
        Eigen::Vector3d p_T;
    
    };
}
#endif