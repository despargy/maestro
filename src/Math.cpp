#include <Math.h>

namespace RCD
{

    Math::Math()
    {
        std::cout<<"MathLibrary Constructor"<<std::endl;
    }

    Math::~Math()
    {
        std::cout<<"MathLibrary De-Constructor"<<std::endl;
    }
    Eigen::Matrix3d Math::scewSymmetric(Eigen::Vector3d t)
    {
        Eigen::Matrix3d t_hat;
        t_hat << 0, -t(2), t(1),
            t(2), 0, -t(0),
            -t(1), t(0), 0;
        return t_hat;
    }
    Eigen::Vector3d Math::get_pDesiredTrajectory(Eigen::Vector3d p_d0_, double dt)
    {
        Eigen::Vector3d p_d;
        // desired position of t_now = dt = time_elapsed
        // p_d(0) = p_d0_(0) + 0.1*sin(2*M_PI*this->b_coef*dt); 
        // p_d(1) = p_d0_(1) + 0.1*cos(2*M_PI*this->b_coef*dt); 
        // p_d(2) = p_d0_(2) + 0.0;

        // x, z axis TODO extra change id od static axis 
        p_d(0) = p_d0_(0) + 0.1*sin(2*M_PI*0.1*dt); 
        p_d(1) = p_d0_(1) + 0.0; 
        p_d(2) = p_d0_(2) + 0.1*cos(2*M_PI*0.1*dt);
        return p_d;
    }
    Eigen::Vector3d Math::get_dpDesiredTrajectory(Eigen::Vector3d p_d0_,Eigen::Vector3d p_d_cur, double dt)
    {
        Eigen::Vector3d p_d_next;
        p_d_next = get_pDesiredTrajectory(p_d0_, dt);
        Eigen::Vector3d dp_d;
        dp_d = (p_d_next - p_d_cur)/dt;
        dp_d(1) = 0; // axis id which axis traj is static
        return dp_d;
    }  
    Eigen::Vector3d Math::get_ddpDesiredTrajectory(Eigen::Vector3d p_d0_,Eigen::Vector3d p_d_cur,Eigen::Vector3d dp_d_cur, double dt)
    {
        Eigen::Vector3d dp_d_next;
        dp_d_next = get_dpDesiredTrajectory(p_d0_, p_d_cur, dt);
        Eigen::Vector3d ddp_d;
        ddp_d = (dp_d_next - dp_d_cur)/dt;
        ddp_d(1) = 0; // axis id which axis traj is static
        return ddp_d;
    }    

    // Orientation
    Eigen::Matrix3d Math::get_RDesiredOrientation(Eigen::Quaterniond Q_0, double dt)
    {
        Eigen::Quaterniond temp = Q_0;
        // desired orientation of t_now = dt = time_elapsed
        temp.x() = Q_0.x() + 0.2*cos(2*0.1*M_PI*dt);
        temp.normalize();
        return temp.toRotationMatrix(); 
    }
    // Eigen::Matrix3d Math::get_RDesiredOrientation(uble dt)
    // {
    //     Eigen::Vector3d v_sx(1,0,0);
    //     Eigen::Matrix3d dR_d;

    //     return dR_d;
    // }
}