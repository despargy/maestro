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
    Eigen::Vector3d Math::scewSymmetricInverse(Eigen::Matrix3d m)
    {
        return Eigen::Vector3d( (m(2,1)-m(1,2))/2.0, (m(0,2) - m(2,0))/2.0, (m(1,0) - m(0,1))/2.0 );
    }
    Eigen::Vector3d Math::deriv_RcRdTwd(Eigen::Vector3d RcRdTwd_prev,Eigen::Vector3d RcRdTwd_cur, double dt)
    {
        return  (RcRdTwd_cur - RcRdTwd_prev)/dt;
    }
    Eigen::Vector3d Math::get_pDesiredTrajectory(Eigen::Vector3d p_d0_, double dt)
    {
        Eigen::Vector3d p_d;
        double freq = 0.3;
        // desired position of t_now = dt = time_elapsed
        p_d(0) = p_d0_(0) + (0.06-0.06*cos(2*M_PI*freq*dt)); 
        p_d(1) = p_d0_(1) + 0.02*sin(2*M_PI*freq*dt); 
        p_d(2) = p_d0_(2) + 0.0;

        // x, z axis TODO extra change id od static axis 
        // p_d(0) = p_d0_(0) - 0.1*sin(2*M_PI*0.3*dt); 
        // p_d(1) = p_d0_(1) + 0.0; 
        // p_d(2) = p_d0_(2) + 0.1*cos(2*M_PI*0.25*dt) - 0.1;

        // x, z axis TODO extra change id od static axis 
        // p_d(0) = p_d0_(0) - 0.1*sin(2*M_PI*0.1*dt); 
        // p_d(1) = p_d0_(1) + 0.0; 
        // p_d(2) = p_d0_(2) + 0.1*cos(2*M_PI*0.1*dt) ;
        return p_d;
    }
    Eigen::Vector3d Math::get_dpDesiredTrajectory(Eigen::Vector3d p_d0_,Eigen::Vector3d p_d_cur, double dt)
    {
        Eigen::Vector3d p_d_next;
        p_d_next = get_pDesiredTrajectory(p_d0_, dt);
        Eigen::Vector3d dp_d;
        dp_d = (p_d_next - p_d_cur)/dt;
        //dp_d(1) = 0; // axis id which axis traj is static
       // dp_d(2) = 0; // axis id which axis traj is static

        return dp_d;
    }  
    Eigen::Vector3d Math::get_ddpDesiredTrajectory(Eigen::Vector3d p_d0_,Eigen::Vector3d p_d_cur,Eigen::Vector3d dp_d_cur, double dt)
    {
        Eigen::Vector3d dp_d_next;
        dp_d_next = get_dpDesiredTrajectory(p_d0_, p_d_cur, dt);
        Eigen::Vector3d ddp_d;
        ddp_d = (dp_d_next - dp_d_cur)/dt;
       // ddp_d(1) = 0; // axis id which axis traj is static
       // ddp_d(2) = 0; // axis id which axis traj is static

        return ddp_d;
    }    

    // Orientation
    Eigen::Matrix3d Math::get_RDesiredRotationMatrix(Eigen::Quaterniond Q_0, double dt)
    {
        Eigen::Quaterniond temp = Q_0;
        // desired orientation of t_now = dt = time_elapsed
        temp.x() = Q_0.x() + 0.2*sin(2*0.1*M_PI*dt);
        temp.normalize();
        return temp.toRotationMatrix(); 
    }
    Eigen::Matrix3d Math::get_dRDesiredRotationMatrix(Eigen::Quaterniond Q_0, Eigen::Matrix3d R_cur,double dt)
    {
        return (get_RDesiredRotationMatrix(Q_0, dt) - R_cur) / dt;
    }
    Eigen::Vector3d Math::get_dp_CoM(Eigen::Vector3d com_p_prev,Eigen::Vector3d com_p_cur,double dt)
    {
        return (com_p_cur - com_p_prev )/dt;
    }

    Eigen::Matrix3d Math::get_dR_CoM(Eigen::Matrix3d R_CoM_prev,Eigen::Matrix3d R_CoM_cur, double dt)
    {
        return (R_CoM_cur - R_CoM_prev)/dt;
    }

}