#include <Eigen/Dense>
#include <unitree_legged_msgs/LowState.h>
#include <Leg.h>

#ifndef _DATAHANDLER_H_
#define _DATAHANDLER_H_

namespace RCD
{
    class DataHandler
    {
    public:
        
        struct LogStructure
        {
            /* data */
            int *a;
            // ------ Logging Robot class ------ //
            Eigen::Vector3d *p_c;
            Eigen::VectorXd *F_a;
            Eigen::MatrixXd *R_c;
            // vvvv, F_c
            // ------                       ------ //
            // ------ Logging Leg class ------ //
            Eigen::MatrixXd *J0, *J1, *J2, *J3;
            struct LegDataProfile
            {
                int *id;
                Eigen::MatrixXd *J;
                Eigen::Vector3d *tau;
                double *prob_stab;
            }LegsProfile[4];
            // q 
            // Eigen::Vector3d tau[4];
            // double prob_stab[4];
            // ------                       ------ //
            // ------ Logging Controller class ------ //

            // ------                       ------ //
            // ev, eo, ep, t_real, tv, d_tv

        }log_data;

        DataHandler();
        ~DataHandler();

        void test_pointersLogData();
    };
}

#endif