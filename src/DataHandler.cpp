#include <DataHandler.h>

namespace RCD
{
    DataHandler::DataHandler()
    {
        std::cout<<"DataHandler Constructor"<<std::endl;
        this->main_path = "/home/despargy/go1_ws/src/maestro/CSV/";
    }
    DataHandler::~DataHandler()
    {
        std::cout<<"DataHandler De-Constructor"<<std::endl;
    }
    void DataHandler::open()
    {
        logfile_errors.open(main_path + "Errors.csv");
        logfile_errors 
                // time, virtual, d_virtual
                << "t_real" << "\t"
                << "tv" << "\t"
                << "d_tv" << "\t"
                // e_p
                << "e_p_x"<<"\t"<< "e_p_y"<<"\t"<< "e_p_z"  <<"\t"
                //e_o
                << "e_o_x"<<"\t"<< "e_o_y"<<"\t"<< "e_o_z"  <<"\t"
                //e_v
                << "e_v_lin_x"<<"\t"<< "e_v_lin_y"<<"\t"<< "e_v_lin_z"  <<"\t"
                << "e_v_ang_x"<<"\t"<< "e_v_ang_y"<<"\t"<< "e_v_ang_z"  << "\n";

        logfile_weights.open(main_path + "Weights.csv");
        logfile_weights
                // time, virtual, d_virtual
                << "t_real" << "\t"
                << "tv" << "\t"
                << "d_tv" << "\t"
                // Weights 12
                << "W_0x"<<"\t"<< "W_0y"<<"\t"<< "W_0z"  <<"\t"                
                << "W_1x"<<"\t"<< "W_1y"<<"\t"<< "W_1z"  <<"\t"                
                << "W_2x"<<"\t"<< "W_2y"<<"\t"<< "W_2z"  <<"\t"                
                << "W_3x"<<"\t"<< "W_3y"<<"\t"<< "W_3z"  <<"\n" ;
        logfile_forces.open(main_path + "Forces.csv");               
        logfile_forces
                // time, virtual, d_virtual
                << "t_real" << "\t"
                << "tv" << "\t"
                << "d_tv" << "\t"
                // F_c 6 // TODO check representation names
                << "f_C_x"<<"\t"<< "f_C_y"<<"\t"<< "f_C_z"  <<"\t"                
                << "tau_C_x"<<"\t"<< "tau_C_y"<<"\t"<< "tau_C_z"  <<"\t"                    
                // F_a 12 
                << "Fa_0x"<<"\t"<< "Fa_0y"<<"\t"<< "Fa_0z"  <<"\t"                
                << "Fa_1x"<<"\t"<< "Fa_1y"<<"\t"<< "Fa_1z"  <<"\t"                
                << "Fa_2x"<<"\t"<< "Fa_2y"<<"\t"<< "Fa_2z"  <<"\t"                
                << "Fa_3x"<<"\t"<< "Fa_3y"<<"\t"<< "Fa_3z"  <<"\n" ; 
        logfile_com.open(main_path + "CoM.csv");               
        logfile_com
                // time, virtual, d_virtual
                << "t_real" << "\t"
                << "tv" << "\t"
                << "d_tv" << "\t"
                // p_c
                << "p_c_x"<<"\t"<< "p_c_y"<<"\t"<< "p_c_z"  <<"\t" 
                // R_c
                << "Rc_00"<<"\t"<< "Rc_01"<<"\t"<< "Rc_02"  <<"\t"                
                << "Rc_10"<<"\t"<< "Rc_11"<<"\t"<< "Rc_12"  <<"\t"                
                << "Rc_20"<<"\t"<< "Rc_21"<<"\t"<< "Rc_22"  <<"\t"   
                // p_d
                << "p_d_x"<<"\t"<< "p_d_y"<<"\t"<< "p_d_z"  <<"\t" 
                // R_d
                << "Rd_00"<<"\t"<< "Rd_01"<<"\t"<< "Rd_02"  <<"\t"                
                << "Rd_10"<<"\t"<< "Rd_11"<<"\t"<< "Rd_12"  <<"\t"                
                << "Rd_20"<<"\t"<< "Rd_21"<<"\t"<< "Rd_22"  <<"\n"   ;

        logfile_legs.open(main_path + "Legs.csv");
        logfile_legs
                    // time, virtual, d_virtual
                    << "t_real" << "\t"
                    << "tv" << "\t"
                    << "d_tv" << "\t";
        for(int i =0; i < 4 ; i++)
        {
            logfile_legs
                    // J 6x3
                    << "J_00"<<"\t"<< "J_01"<<"\t"<< "J_02"  <<"\t"                
                    << "J_10"<<"\t"<< "J_11"<<"\t"<< "J_12"  <<"\t"                
                    << "J_20"<<"\t"<< "J_21"<<"\t"<< "J_22"  <<"\t"       
                    << "J_30"<<"\t"<< "J_31"<<"\t"<< "J_32"  <<"\t"                
                    << "J_40"<<"\t"<< "J_41"<<"\t"<< "J_42"  <<"\t"                
                    << "J_50"<<"\t"<< "J_51"<<"\t"<< "J_52"  <<"\t" 
                    // tau 3 vecotr for each joint
                    << "tau_hip"<<"\t"<< "tau_thigh"<<"\t"<< "tau_calf"  <<"\t" 
                    //prob stable
                    << "prob_stable" << "\t"
                    // q joints TODO OK for KDL?
                    << "q_hip"<<"\t"<< "q_thigh"<<"\t"<< "q_calf"  <<"\t"; 
        }               
        logfile_legs << "\n";
    }

    void DataHandler::logNow()
    {
        this->logErros();   
        this->logWeights();
        this->logForces();
        this->logCoM();
        this->logLegs();
    }
    void DataHandler::close()
    {
        logfile_errors.close();
        logfile_weights.close();
        logfile_forces.close();
        logfile_com.close();
        logfile_legs.close();

    }
    void DataHandler::logLegs()
    {
        logfile_legs
                    // time, virtual, d_virtual
                    << *log_data.t_real  <<"\t"
                    << *log_data.tv  <<"\t"
                    << *log_data.d_tv  <<"\t";
        for(int i =0; i < 4 ; i++)
        {
                logfile_legs
                    // J 6x3
                    <<  (*log_data.LegsProfile[i].J)(0,0)<<"\t"<< (*log_data.LegsProfile[i].J)(0,1)<<"\t"<< (*log_data.LegsProfile[i].J)(0,2)  <<"\t"                
                    <<  (*log_data.LegsProfile[i].J)(1,0)<<"\t"<< (*log_data.LegsProfile[i].J)(1,1)<<"\t"<< (*log_data.LegsProfile[i].J)(1,2)  <<"\t"                
                    <<  (*log_data.LegsProfile[i].J)(2,0)<<"\t"<< (*log_data.LegsProfile[i].J)(2,1)<<"\t"<< (*log_data.LegsProfile[i].J)(2,2)  <<"\t"       
                    <<  (*log_data.LegsProfile[i].J)(3,0)<<"\t"<< (*log_data.LegsProfile[i].J)(3,1)<<"\t"<< (*log_data.LegsProfile[i].J)(3,2)  <<"\t"                
                    <<  (*log_data.LegsProfile[i].J)(4,0)<<"\t"<< (*log_data.LegsProfile[i].J)(4,1)<<"\t"<< (*log_data.LegsProfile[i].J)(4,2)  <<"\t"                
                    <<  (*log_data.LegsProfile[i].J)(5,0)<<"\t"<< (*log_data.LegsProfile[i].J)(5,1)<<"\t"<< (*log_data.LegsProfile[i].J)(5,2)  <<"\t" 
                    // tau 3 vecotr for each joint
                    << (*log_data.LegsProfile[i].tau)(0)<<"\t"<<  (*log_data.LegsProfile[i].tau)(1)<<"\t"<<  (*log_data.LegsProfile[i].tau)(2)  <<"\t" 
                    //prob stable
                    << *log_data.LegsProfile[i].prob_stab << "\t"
                    // q joints
                    << (*log_data.LegsProfile[i].q)(0)<<"\t"<< (*log_data.LegsProfile[i].q)(1)<<"\t"<<  (*log_data.LegsProfile[i].q)(2)  <<"\t"; 
        }               
        logfile_legs << "\n";
    }
    void DataHandler::logCoM()
    {
        logfile_com
                // time, virtual, d_virtual
                << *log_data.t_real  <<"\t"
                << *log_data.tv  <<"\t"
                << *log_data.d_tv  <<"\t"
                // p_c
                << (*log_data.p_c)(0)  <<"\t" << (*log_data.p_c)(1)  <<"\t" << (*log_data.p_c)(2)  <<"\t"
                // Rc 12 
                << (*log_data.R_c)(0,0)<< "\t" << (*log_data.R_c)(0,1)<< "\t" << (*log_data.R_c)(0,2)<< "\t"
                << (*log_data.R_c)(1,0)<< "\t" << (*log_data.R_c)(1,1)<< "\t" << (*log_data.R_c)(1,2)<< "\t"
                << (*log_data.R_c)(2,0)<< "\t" << (*log_data.R_c)(2,1)<< "\t" << (*log_data.R_c)(2,2)<< "\t" 
                // Desired p_d
                << (*log_data.p_d)(0)  <<"\t" << (*log_data.p_d)(1)  <<"\t" << (*log_data.p_d)(2)  <<"\t"      
                // Rd 12 
                << (*log_data.R_d)(0,0)<< "\t" << (*log_data.R_d)(0,1)<< "\t" << (*log_data.R_d)(0,2)<< "\t"
                << (*log_data.R_d)(1,0)<< "\t" << (*log_data.R_d)(1,1)<< "\t" << (*log_data.R_d)(1,2)<< "\t"
                << (*log_data.R_d)(2,0)<< "\t" << (*log_data.R_d)(2,1)<< "\t" << (*log_data.R_d)(2,2)<< "\n";           
    }
    void DataHandler::logWeights()
    {
        logfile_weights 
                        // time, virtual, d_virtual
                        << *log_data.t_real  <<"\t"
                        << *log_data.tv  <<"\t"
                        << *log_data.d_tv  <<"\t"
                        // Vector 12 weights
                        << (*log_data.vvvv)(0)<< "\t" << (*log_data.vvvv)(1)<< "\t" << (*log_data.vvvv)(2)<< "\t"
                        << (*log_data.vvvv)(3)<< "\t" << (*log_data.vvvv)(4)<< "\t" << (*log_data.vvvv)(5)<< "\t"
                        << (*log_data.vvvv)(6)<< "\t" << (*log_data.vvvv)(7)<< "\t" << (*log_data.vvvv)(8)<< "\t"
                        << (*log_data.vvvv)(9)<< "\t" << (*log_data.vvvv)(10)<< "\t" << (*log_data.vvvv)(11)<< "\n";
    }
    void DataHandler::logErros()
    {                                   
                        
        logfile_errors 
                // time, virtual, d_virtual
                << *log_data.t_real  <<"\t"
                << *log_data.tv  <<"\t"
                << *log_data.d_tv  <<"\t"
                // e_p
                << (*log_data.e_p)(0)  <<"\t" << (*log_data.e_p)(1)  <<"\t" << (*log_data.e_p)(2)  <<"\t"
                // e_o
                << (*log_data.e_o)(0)  <<"\t" << (*log_data.e_o)(1)  <<"\t" << (*log_data.e_o)(2)  <<"\t"
                // e_v
                << (*log_data.e_v)(0)  <<"\t" << (*log_data.e_v)(1)  <<"\t" << (*log_data.e_v)(2)  <<"\t"
                << (*log_data.e_v)(3)  <<"\t" << (*log_data.e_v)(4)  <<"\t" << (*log_data.e_v)(5)  << "\n";
    }
    void DataHandler::logForces()
    {
        logfile_forces
                        // time, virtual, d_virtual
                        << *log_data.t_real  <<"\t"
                        << *log_data.tv  <<"\t"
                        << *log_data.d_tv  <<"\t"
                        // // Fc vector 6 
                        << (*log_data.F_c)(0)<< "\t" << (*log_data.F_c)(1)<< "\t" << (*log_data.F_c)(2)<< "\t"
                        << (*log_data.F_c)(3)<< "\t" << (*log_data.F_c)(4)<< "\t" << (*log_data.F_c)(5)<< "\t"
                        // Fa vector 12 
                        << (*log_data.F_a)(0)<< "\t" << (*log_data.F_a)(1)<< "\t" << (*log_data.F_a)(2)<< "\t"
                        << (*log_data.F_a)(3)<< "\t" << (*log_data.F_a)(4)<< "\t" << (*log_data.F_a)(5)<< "\t"
                        << (*log_data.F_a)(6)<< "\t" << (*log_data.F_a)(7)<< "\t" << (*log_data.F_a)(8)<< "\t"
                        << (*log_data.F_a)(9)<< "\t" << (*log_data.F_a)(10)<< "\t" << (*log_data.F_a)(11)<< "\n";
    }
}


//TODO d_p,d_w d_R etc.