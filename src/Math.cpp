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
    Eigen::Vector3d Math::get_pDesiredTrajectory(Eigen::Vector3d p_d0_, double t_real)
    {
        Eigen::Vector3d p_d;

        // x,y axis
        double freq = 0.5; // sim 0.65
        // double freq = 0.4; // real

        // p_d(0) = p_d0_(0) + (0.07-0.07*cos(2*M_PI*freq*t_real)); 
        // p_d(1) = p_d0_(1) - 0.07*sin(2*M_PI*freq*t_real); 
        // p_d(2) = p_d0_(2) ;//+ 0.0;

        //  THIS WITH Q_0.x() + 0.2*cos(2*0.25*M_PI*t_real);
        // p_d(0) = p_d0_(0) - 0.07*(sin(2*M_PI*freq*t_real)); 
        // p_d(1) = p_d0_(1) - 0.07*cos(2*M_PI*freq*t_real); 
        // p_d(2) = p_d0_(2) ;//+ 0.0;

        // p_d(0) = p_d0_(0) - 0.04*(sin(2*M_PI*freq*t_real));  //0.07
        // p_d(1) = p_d0_(1) ;//- 0.05*cos(2*M_PI*freq*t_real);  //0.07
        // p_d(2) = p_d0_(2) + 0.0;

        // x, z axis 
        // p_d(0) = p_d0_(0) - 0.1*sin(2*M_PI*0.3*dt); 
        // p_d(1) = p_d0_(1) + 0.0; 
        // p_d(2) = p_d0_(2) + 0.1*cos(2*M_PI*0.25*dt) - 0.1;

        // p_d = p_d0_ + (this->p_T - p_d0_)*(1- std::exp(-0.5*t_real));
        // target 

 

        // INF. w0 
        p_d(0) = p_d0_(0) + 0.02*sin(2*M_PI*freq*t_real);  // freq 0.5
        p_d(1) = p_d0_(1) ; 
        p_d(2) = p_d0_(2) ;

        // // 2 front slip , with and without ori Q_0.x();// + 0.2*sin(2*0.55*M_PI*t_real); // ori
        // p_d(0) = p_d0_(0) + 0.05*sin(2*M_PI*freq*t_real);  // freq=0.65
        // p_d(1) = p_d0_(1) ; 
        // p_d(2) = p_d0_(2) - (0.02 - 0.02*cos(2*M_PI*freq*t_real));


        return p_d;
    }
    Eigen::Vector3d Math::get_dpDesiredTrajectory(Eigen::Vector3d p_d0_,Eigen::Vector3d p_d_cur, double dt, double t_real)
    {
        return (get_pDesiredTrajectory(p_d0_, t_real) - p_d_cur)/dt;
    }  
    Eigen::Vector3d Math::get_ddpDesiredTrajectory(Eigen::Vector3d p_d0_,Eigen::Vector3d p_d_cur,Eigen::Vector3d dp_d_cur, double dt, double t_real)
    {

        return ( get_dpDesiredTrajectory(p_d0_, p_d_cur, dt, t_real) - dp_d_cur)/dt;
    }    

    // Orientation
    Eigen::Matrix3d Math::get_RDesiredRotationMatrix(Eigen::Quaterniond Q_0, double t_real)
    {
        Eigen::Quaterniond temp = Q_0;
        // temp.x() = Q_0.x() - 0.15*sin(2*0.55*M_PI*t_real); // ori

        // temp.y() = Q_0.y() ;//+ 0.1*sin(2*0.7*M_PI*t_real); // test
        // temp.z() = Q_0.z()  ;//- 0.1*cos(2*0.3*M_PI*t_real); // test

        temp.normalize();
        return temp.toRotationMatrix(); 
    }
    Eigen::Matrix3d Math::get_dRDesiredRotationMatrix(Eigen::Quaterniond Q_0, Eigen::Matrix3d R_cur,double dt, double t_real)
    {
        return (get_RDesiredRotationMatrix(Q_0, t_real) - R_cur)/ dt;
    }
    Eigen::Vector3d Math::get_dp_CoM(Eigen::Vector3d com_p_prev,Eigen::Vector3d com_p_cur,double dt)
    {
        return (com_p_cur - com_p_prev )/dt;
    }

    Eigen::Matrix3d Math::get_dR_CoM(Eigen::Matrix3d R_CoM_prev,Eigen::Matrix3d R_CoM_cur, double dt)
    {
        return (R_CoM_cur - R_CoM_prev)/dt;
    }
    double Math::exp_inf()
    {
        return exp(std::numeric_limits<double>::infinity());
    }
}