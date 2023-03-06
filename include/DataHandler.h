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

        std::ofstream logfile;
        std::string main_path; 
        struct LogStructure
        {
            /* data */
            // ------ Logging Robot class ------ //
            Eigen::Vector3d *p_c;
            Eigen::VectorXd *vvvv;
            // ------                       ------ //
            
            // ------ Logging Desired ------ //
            Eigen::Vector3d *p_d;
            // ------                       ------ //

            // ------ Logging Leg class ------ //
            double *leg_prob_0, *leg_prob_1, *leg_prob_2, *leg_prob_3;
            // ------                       ------ //

            // ------ Logging Controller class ------ //
            Eigen::Vector3d *e_p, *e_o;
            Eigen::VectorXd *e_v;
            double *t_real, *tv, *d_tv;
            // ------                       ------ //

        }log_data;

        DataHandler();
        ~DataHandler();
        
        // void* keepWritting(void* args);

        void logData();
        void openOnce();
        void closeOnce();
    };
}

#endif