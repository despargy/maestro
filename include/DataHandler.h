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
            float *leg_0_x, *leg_0_y, *leg_0_z;
            float *leg_1_x, *leg_1_y, *leg_1_z;
            float *leg_2_x, *leg_2_y, *leg_2_z;
            float *leg_3_x, *leg_3_y, *leg_3_z;

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
        void logDataTips();
        void openOnce();
        void openOnceTips();
        void closeOnce();
    };
}

#endif