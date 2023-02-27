#include <Eigen/Dense>
#include <unitree_legged_msgs/LowState.h>
#include <Leg.h>
#include <iostream>
#include <fstream>
#include<vector>
#include <string>

#ifndef _DATAHANDLER_H_
#define _DATAHANDLER_H_

namespace RCD
{
    class DataHandler
    {
    public:

    	//https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
	    // const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");

        std::ofstream logfile_errors, logfile_weights, logfile_forces, logfile_com, logfile_legs;
        std::string main_path; 

        struct LogStructure
        {
            /* data */
            // ------ Logging Robot class ------ //
            Eigen::Vector3d *p_c;
            Eigen::VectorXd *F_a, *F_c;
            Eigen::MatrixXd *R_c;
            Eigen::VectorXd *vvvv;
            // ------                       ------ //
            // ------ Logging Leg class ------ //
            struct LegDataProfile
            {
                int *id;
                Eigen::MatrixXd *J;
                Eigen::Vector3d *tau;
                double *prob_stab;
                KDL::JntArray *q;
            }LegsProfile[4];
            // ------                       ------ //
            // ------ Logging Controller class ------ //
            Eigen::Vector3d *e_p, *e_o;
            Eigen::VectorXd *e_v;
            double *t_real, *tv, *d_tv;
            // ------                       ------ //
            // ------ Logging Desired ------ //
            Eigen::Vector3d *p_d;
            Eigen::Matrix3d *R_d;
            // ------                       ------ //
        }log_data;

        DataHandler();
        ~DataHandler();
        void open();
        void logNow();
        void logErros();
        void logWeights();
        void logForces();
        void logCoM();
        void logLegs();

        void close();
    };
}

#endif