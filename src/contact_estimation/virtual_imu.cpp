#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/AccelStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"

#include <vector>

geometry_msgs::AccelStamped fl_imu,fr_imu,rl_imu,rr_imu; 

std::vector<geometry_msgs::Twist> curr_vel(4),prev_vel(4);// Linear velocities


bool gazebo_flag{false};

void gazebo_cb(const gazebo_msgs::LinkStates::ConstPtr & msg)
{   
    gazebo_flag = true;
    curr_vel[0] = msg->twist[8];   // FL 
    curr_vel[1] = msg->twist[12];  // FR
    curr_vel[2] = msg->twist[16];  // RL
    curr_vel[3] = msg->twist[20];  // RR
}


void initialize_msgs()
{
    fl_imu.header.frame_id = "FL_foot";
    fr_imu.header.frame_id = "FR_foot";
    rl_imu.header.frame_id = "RL_foot";
    rr_imu.header.frame_id = "RR_foot";


    fl_imu.accel.linear.x  = 0;
    fl_imu.accel.linear.y  = 0;
    fl_imu.accel.linear.z  = 0;

    fr_imu.accel.linear.x  = 0;
    fr_imu.accel.linear.y  = 0;
    fr_imu.accel.linear.z  = 0;

    rl_imu.accel.linear.x  = 0;
    rl_imu.accel.linear.y  = 0;
    rl_imu.accel.linear.z  = 0;

    rr_imu.accel.linear.x  = 0;
    rr_imu.accel.linear.y  = 0;
    rr_imu.accel.linear.z  = 0;
}

// Returns time in ms
double getTime()
{
    return ros::Time::now().toSec();
}



int main(int argc, char **argv){
    int refresh_rate = 500;
    double dt = 1./500.;

    ros::init(argc,argv,"virtual_imu");

    ros::NodeHandle nh;

    ros::Rate rate(refresh_rate);

    ros::Publisher fl_pub = nh.advertise<geometry_msgs::AccelStamped>("/imu_fl",1);
    ros::Publisher fr_pub = nh.advertise<geometry_msgs::AccelStamped>("/imu_fr",1);
    ros::Publisher rl_pub = nh.advertise<geometry_msgs::AccelStamped>("/imu_rl",1);
    ros::Publisher rr_pub = nh.advertise<geometry_msgs::AccelStamped>("/imu_rr",1);


    ros::Subscriber gazebo_sub = nh.subscribe("/gazebo/link_states", 1, gazebo_cb); 

    double a = 0.001;

    
    

    initialize_msgs();

    while(!gazebo_flag){
        ros::spinOnce();
        rate.sleep();
    }
    prev_vel[0] = curr_vel[0];
    prev_vel[1] = curr_vel[1];
    prev_vel[2] = curr_vel[2];
    prev_vel[3] = curr_vel[3];

    double t1,t2;
    t1 = getTime();
    while (ros::ok()){
        ros::spinOnce();

        if (!gazebo_flag){
            continue;
        }else{
            gazebo_flag = false;
        }
       

        // f   << curr_pose.poses[0].position.x << "," << curr_pose.poses[0].position.y<< ","
        //     << curr_pose.poses[1].position.x << "," << curr_pose.poses[1].position.y<< ","
        //     << curr_pose.poses[2].position.x << "," << curr_pose.poses[2].position.y<< ","
        //     << curr_pose.poses[3].position.x << "," << curr_pose.poses[3].position.y<< "\n";



        fl_imu.header.stamp = ros::Time::now();
        fr_imu.header.stamp = ros::Time::now();
        rl_imu.header.stamp = ros::Time::now();
        rr_imu.header.stamp = ros::Time::now();


        t2 = getTime();
        // dt = t2-t1;
        // if (std::abs(dt) < 0.0000001){
        //     rate.sleep();
        //     continue;
        // }
        // /* Linear Accelerations */

        fl_imu.accel.linear.x  = (1-a)*fl_imu.accel.linear.x +  a*(curr_vel[0].linear.x - prev_vel[0].linear.x)/dt;
        fl_imu.accel.linear.y  = (1-a)*fl_imu.accel.linear.y +  a*(curr_vel[0].linear.y - prev_vel[0].linear.y)/dt;
        fl_imu.accel.linear.z  = (1-a)*fl_imu.accel.linear.z +  a*(curr_vel[0].linear.z - prev_vel[0].linear.z)/dt;


        fr_imu.accel.linear.x  = (1-a)*fr_imu.accel.linear.x +  a*(curr_vel[1].linear.x - prev_vel[1].linear.x)/dt;
        fr_imu.accel.linear.y  = (1-a)*fr_imu.accel.linear.y +  a*(curr_vel[1].linear.y - prev_vel[1].linear.y)/dt;
        fr_imu.accel.linear.z  = (1-a)*fr_imu.accel.linear.z +  a*(curr_vel[1].linear.z - prev_vel[1].linear.z)/dt;




        rl_imu.accel.linear.x  = (1-a)*rl_imu.accel.linear.x +  a*(curr_vel[2].linear.x - prev_vel[2].linear.x)/dt;
        rl_imu.accel.linear.y  = (1-a)*rl_imu.accel.linear.y +  a*(curr_vel[2].linear.y - prev_vel[2].linear.y)/dt;
        rl_imu.accel.linear.z  = (1-a)*rl_imu.accel.linear.z +  a*(curr_vel[2].linear.z - prev_vel[2].linear.z)/dt;


        rr_imu.accel.linear.x  = (1-a)*rr_imu.accel.linear.x +  a*(curr_vel[3].linear.x - prev_vel[3].linear.x)/dt;
        rr_imu.accel.linear.y  = (1-a)*rr_imu.accel.linear.y +  a*(curr_vel[3].linear.y - prev_vel[3].linear.y)/dt;
        rr_imu.accel.linear.z  = (1-a)*rr_imu.accel.linear.z +  a*(curr_vel[3].linear.z - prev_vel[3].linear.z)/dt;

        // fl_imu.accel.linear.x = (curr_vel[0].linear.x - prev_vel[0].linear.x)/dt;
        // fl_imu.accel.linear.y = (curr_vel[0].linear.y - prev_vel[0].linear.y)/dt;
        // fl_imu.accel.linear.z = (curr_vel[0].linear.z - prev_vel[0].linear.z)/dt;


        // fr_imu.accel.linear.x = (curr_vel[1].linear.x - prev_vel[1].linear.x)/dt;
        // fr_imu.accel.linear.y = (curr_vel[1].linear.y - prev_vel[1].linear.y)/dt;
        // fr_imu.accel.linear.z = (curr_vel[1].linear.z - prev_vel[1].linear.z)/dt;
        
        // rl_imu.accel.linear.x = (curr_vel[2].linear.x - prev_vel[2].linear.x)/dt;
        // rl_imu.accel.linear.y = (curr_vel[2].linear.y - prev_vel[2].linear.y)/dt;
        // rl_imu.accel.linear.z = (curr_vel[2].linear.z - prev_vel[2].linear.z)/dt;
        
        // rr_imu.accel.linear.x = (curr_vel[3].linear.x - prev_vel[3].linear.x)/dt;
        // rr_imu.accel.linear.y = (curr_vel[3].linear.y - prev_vel[3].linear.y)/dt;
        // rr_imu.accel.linear.z = (curr_vel[3].linear.z - prev_vel[3].linear.z)/dt;
        t1 = getTime();
        // /* Angular Velocities */
        // 

        fl_pub.publish(fl_imu);
        fr_pub.publish(fr_imu);
        rl_pub.publish(rl_imu);
        rr_pub.publish(rr_imu);
        
        // // Update previous state
        prev_vel[0] = curr_vel[0];
        prev_vel[1] = curr_vel[1];
        prev_vel[2] = curr_vel[2];
        prev_vel[3] = curr_vel[3];       
        rate.sleep();
    }
}