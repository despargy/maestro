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
        Eigen::Vector3d get_pDesiredTrajectory(Eigen::Vector3d p_d0_, double dt);
        Eigen::Vector3d get_dpDesiredTrajectory(Eigen::Vector3d p_d0_,Eigen::Vector3d p_d_cur, double dt);
        Eigen::Vector3d get_ddpDesiredTrajectory(Eigen::Vector3d p_d0_,Eigen::Vector3d p_d_cur,Eigen::Vector3d dp_d_cur, double dt);
        Eigen::Matrix3d get_RDesiredOrientation(Eigen::Quaterniond Q_0, double dt);

    };
}
#endif