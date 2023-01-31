#include <GazeboInterpreter.h>

GazeboInterpreter::GazeboInterpreter()
{
    ROS_INFO("Constrauctor GazeboInterpreter");
}
GazeboInterpreter::~GazeboInterpreter()
{
    ROS_INFO("De-Constrauctor GazeboInterpreter");
}
void GazeboInterpreter::initGazebo()
{
        start_up = true;

        imu_sub = nh->subscribe("/trunk_imu", 1, &GazeboInterpreter::imuCallback, this);
        footForce_sub[0] = nh->subscribe("/visual/FR_foot_contact/the_force", 1, &GazeboInterpreter::FRfootCallback, this);
        footForce_sub[1] = nh->subscribe("/visual/FL_foot_contact/the_force", 1, &GazeboInterpreter::FLfootCallback, this);
        footForce_sub[2] = nh->subscribe("/visual/RR_foot_contact/the_force", 1, &GazeboInterpreter::RRfootCallback, this);
        footForce_sub[3] = nh->subscribe("/visual/RL_foot_contact/the_force", 1, &GazeboInterpreter::RLfootCallback, this);
        servo_sub[0] = nh->subscribe("/" + robot_name + "_gazebo/FR_hip_controller/state", 1, &GazeboInterpreter::FRhipCallback, this);
        servo_sub[1] = nh->subscribe("/" + robot_name + "_gazebo/FR_thigh_controller/state", 1, &GazeboInterpreter::FRthighCallback, this);
        servo_sub[2] = nh->subscribe("/" + robot_name + "_gazebo/FR_calf_controller/state", 1, &GazeboInterpreter::FRcalfCallback, this);
        servo_sub[3] = nh->subscribe("/" + robot_name + "_gazebo/FL_hip_controller/state", 1, &GazeboInterpreter::FLhipCallback, this);
        servo_sub[4] = nh->subscribe("/" + robot_name + "_gazebo/FL_thigh_controller/state", 1, &GazeboInterpreter::FLthighCallback, this);
        servo_sub[5] = nh->subscribe("/" + robot_name + "_gazebo/FL_calf_controller/state", 1, &GazeboInterpreter::FLcalfCallback, this);
        servo_sub[6] = nh->subscribe("/" + robot_name + "_gazebo/RR_hip_controller/state", 1, &GazeboInterpreter::RRhipCallback, this);
        servo_sub[7] = nh->subscribe("/" + robot_name + "_gazebo/RR_thigh_controller/state", 1, &GazeboInterpreter::RRthighCallback, this);
        servo_sub[8] = nh->subscribe("/" + robot_name + "_gazebo/RR_calf_controller/state", 1, &GazeboInterpreter::RRcalfCallback, this);
        servo_sub[9] = nh->subscribe("/" + robot_name + "_gazebo/RL_hip_controller/state", 1, &GazeboInterpreter::RLhipCallback, this);
        servo_sub[10] = nh->subscribe("/" + robot_name + "_gazebo/RL_thigh_controller/state", 1, &GazeboInterpreter::RLthighCallback, this);
        servo_sub[11] = nh->subscribe("/" + robot_name + "_gazebo/RL_calf_controller/state", 1, &GazeboInterpreter::RLcalfCallback, this);
    
        pub_gazeboLowState_ = nh->advertise<unitree_legged_msgs::LowState>(sim_lowstate_topic,1);

        sub_gazeboLowCmd_ = nh->subscribe(sim_lowcmd_topic,1, &GazeboInterpreter::LowCmdsCallback, this);
        servo_pub[0] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_hip_controller/command", 1);
        servo_pub[1] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_thigh_controller/command", 1);
        servo_pub[2] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FR_calf_controller/command", 1);
        servo_pub[3] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_hip_controller/command", 1);
        servo_pub[4] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_thigh_controller/command", 1);
        servo_pub[5] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/FL_calf_controller/command", 1);
        servo_pub[6] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_hip_controller/command", 1);
        servo_pub[7] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_thigh_controller/command", 1);
        servo_pub[8] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RR_calf_controller/command", 1);
        servo_pub[9] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_hip_controller/command", 1);
        servo_pub[10] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_thigh_controller/command", 1);
        servo_pub[11] = nh->advertise<unitree_legged_msgs::MotorCmd>("/" + robot_name + "_gazebo/RL_calf_controller/command", 1);



}  

void GazeboInterpreter::imuCallback(const sensor_msgs::Imu & msg)
{ 
    lowState.imu.quaternion[0] = msg.orientation.w;
    lowState.imu.quaternion[1] = msg.orientation.x;
    lowState.imu.quaternion[2] = msg.orientation.y;
    lowState.imu.quaternion[3] = msg.orientation.z;
    lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    
    lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
    
}
void GazeboInterpreter::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[0].mode = msg.mode;
    lowState.motorState[0].q = msg.q;
    lowState.motorState[0].dq = msg.dq;
    lowState.motorState[0].tauEst = msg.tauEst;
}
void GazeboInterpreter::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[1].mode = msg.mode;
    lowState.motorState[1].q = msg.q;
    lowState.motorState[1].dq = msg.dq;
    lowState.motorState[1].tauEst = msg.tauEst;
}
void GazeboInterpreter::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[2].mode = msg.mode;
    lowState.motorState[2].q = msg.q;
    lowState.motorState[2].dq = msg.dq;
    lowState.motorState[2].tauEst = msg.tauEst;
}
void GazeboInterpreter::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[3].mode = msg.mode;
    lowState.motorState[3].q = msg.q;
    lowState.motorState[3].dq = msg.dq;
    lowState.motorState[3].tauEst = msg.tauEst;
}
void GazeboInterpreter::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[4].mode = msg.mode;
    lowState.motorState[4].q = msg.q;
    lowState.motorState[4].dq = msg.dq;
    lowState.motorState[4].tauEst = msg.tauEst;
}
void GazeboInterpreter::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[5].mode = msg.mode;
    lowState.motorState[5].q = msg.q;
    lowState.motorState[5].dq = msg.dq;
    lowState.motorState[5].tauEst = msg.tauEst;
}
void GazeboInterpreter::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[6].mode = msg.mode;
    lowState.motorState[6].q = msg.q;
    lowState.motorState[6].dq = msg.dq;
    lowState.motorState[6].tauEst = msg.tauEst;
}
void GazeboInterpreter::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[7].mode = msg.mode;
    lowState.motorState[7].q = msg.q;
    lowState.motorState[7].dq = msg.dq;
    lowState.motorState[7].tauEst = msg.tauEst;
}
void GazeboInterpreter::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[8].mode = msg.mode;
    lowState.motorState[8].q = msg.q;
    lowState.motorState[8].dq = msg.dq;
    lowState.motorState[8].tauEst = msg.tauEst;
}
void GazeboInterpreter::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    start_up = false;
    lowState.motorState[9].mode = msg.mode;
    lowState.motorState[9].q = msg.q;
    lowState.motorState[9].dq = msg.dq;
    lowState.motorState[9].tauEst = msg.tauEst;
}
void GazeboInterpreter::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[10].mode = msg.mode;
    lowState.motorState[10].q = msg.q;
    lowState.motorState[10].dq = msg.dq;
    lowState.motorState[10].tauEst = msg.tauEst;
}
void GazeboInterpreter::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    lowState.motorState[11].mode = msg.mode;
    lowState.motorState[11].q = msg.q;
    lowState.motorState[11].dq = msg.dq;
    lowState.motorState[11].tauEst = msg.tauEst;
}
void GazeboInterpreter::FRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    lowState.eeForce[0].x = msg.wrench.force.x;
    lowState.eeForce[0].y = msg.wrench.force.y;
    lowState.eeForce[0].z = msg.wrench.force.z;
    lowState.footForce[0] = msg.wrench.force.z;
}
void GazeboInterpreter::FLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    lowState.eeForce[1].x = msg.wrench.force.x;
    lowState.eeForce[1].y = msg.wrench.force.y;
    lowState.eeForce[1].z = msg.wrench.force.z;
    lowState.footForce[1] = msg.wrench.force.z;
}
void GazeboInterpreter::RRfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    lowState.eeForce[2].x = msg.wrench.force.x;
    lowState.eeForce[2].y = msg.wrench.force.y;
    lowState.eeForce[2].z = msg.wrench.force.z;
    lowState.footForce[2] = msg.wrench.force.z;
}
void GazeboInterpreter::RLfootCallback(const geometry_msgs::WrenchStamped& msg)
{
    lowState.eeForce[3].x = msg.wrench.force.x;
    lowState.eeForce[3].y = msg.wrench.force.y;
    lowState.eeForce[3].z = msg.wrench.force.z;
    lowState.footForce[3] = msg.wrench.force.z;
}
void GazeboInterpreter::LowCmdsCallback(const unitree_legged_msgs::LowCmd& msg)
{
    for(int m=0; m<12; m++){
        servo_pub[m].publish(msg.motorCmd[m]);
    }
    ros::spinOnce();
    // usleep(1000);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "maestro_gazebo_interpreter");
    // ros::Rate loop_rate(500);

    GazeboInterpreter gi;
    gi.robot_name = "go1";

    ros::AsyncSpinner spinner(0); // one threads
    spinner.start();

    usleep(1/3); // must wait 300ms, to get first state

    gi.nh = new ros::NodeHandle ;
    gi.initGazebo();

    // motion_init();

    while (ros::ok())
    {
        // Keep publishing low state of Gazebo go1
        gi.pub_gazeboLowState_.publish(gi.lowState);
        // ros::spinOnce();
        // loop_rate.sleep();

    }
    return 0;

}
