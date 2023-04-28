#! /usr/bin/env python3

# ROS libraries
import rospy

# DESPINA : add the correct IMU and FORCE rosmsgs
from sensor_msgs.msg import Imu      # IMU msg
from std_msgs.msg import Float32 # Contact probability msg
from geometry_msgs.msg import AccelStamped
from copy import copy
# Generic libraries
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import KernelDensity

path =  '/home/despargy/go1_ws/src/maestro/src/contact_estimation/contacts/params.csv'
a = np.loadtxt(path,delimiter=",", dtype=str)

a = np.delete(a,0,axis = 0)

a = a.astype(float)

ax_threshold = a[0,0]
ay_threshold = a[0,1]
az_threshold = a[0,2]
desired_batch_size = int(a[0,3])

class pce:
    def __init__(self,topic, batch_size,eval_samples):

        self.topic = topic
        self.batch_size = batch_size
        self.eval_samples = eval_samples

        # (Number of legs, batch_size , feature dimension)
        self.data_batch = np.empty((self.batch_size,2)) # Batch of data to made predictions on


        rospy.init_node('pce',anonymous=True)
        
        pub = rospy.Publisher('/rl_contact_state',Float32,queue_size=1)
       
        p_stable = Float32()

        self.initialize_batch()




        while not rospy.is_shutdown():            
            rl_imu_msg   = rospy.wait_for_message(self.topic, AccelStamped , timeout=None)

            self.data_batch = self.store_new_msg(self.data_batch,rl_imu_msg)

            p_stable.data = self.stable_contact_estimation(self.data_batch)
            pub.publish(p_stable)



    def initialize_batch(self):
        
        print("Batch Initialization")

        for i in range(self.batch_size):
            rl_imu_msg   = rospy.wait_for_message(self.topic, AccelStamped , timeout=None)

            self.data_batch[i,0] =  rl_imu_msg.accel.linear.x
            self.data_batch[i,1] =  rl_imu_msg.accel.linear.y 
            # self.data_batch[i,2] =  rl_imu_msg.accel.linear.z
            # self.data_batch[i,3] =  rl_imu_msg.angular_velocity.x
            # self.data_batch[i,4] =  rl_imu_msg.angular_velocity.y
            # self.data_batch[i,5] =  rl_imu_msg.angular_velocity.z

    def store_new_msg(self,data_batch,rl_imu_msg):

        self.data_batch = np.delete(data_batch,0,axis = 0)
        data_point = np.empty((1,2))
        data_point[0,0] =  rl_imu_msg.accel.linear.x
        data_point[0,1] =  rl_imu_msg.accel.linear.y
        # data_point[0,2] =  rl_imu_msg.accel.linear.z
        # data_point[0,3] =  rl_imu_msg.angular_velocity.x
        # data_point[0,4] =  rl_imu_msg.angular_velocity.y
        # data_point[0,5] =  rl_imu_msg.angular_velocity.z
        self.data_batch  = np.vstack((self.data_batch,data_point))

        return self.data_batch

    def stable_contact_estimation(self,data_batch):
        # Parameters (These are estimated experimentally ONCE during normal gait with stable contact)
        thres_ax = ax_threshold   # In g-scale of IMU 
        thres_ay = ay_threshold
        thres_az = az_threshold
        # thres_wx = 0.25
        # thres_wy = 0.7
        # thres_wz = 0.1
        std_a = 0.001
        std_w = 0.01653 


        kde = np.empty((2,),dtype=object)
        probs = np.empty((3,))

        kde[0] = KernelDensity(bandwidth=std_a, kernel='gaussian').fit(self.data_batch[:,0].reshape((self.batch_size,1)))
        kde[1] = KernelDensity(bandwidth=std_a, kernel='gaussian').fit(self.data_batch[:,1].reshape((self.batch_size,1)))
        # kde[2] = KernelDensity(bandwidth=std_a, kernel='gaussian').fit(self.data_batch[:,2].reshape((self.batch_size,1)))
        # kde[3] = KernelDensity(bandwidth=std_w, kernel='gaussian').fit(self.data_batch[:,3].reshape((self.batch_size,1)))
        # kde[4] = KernelDensity(bandwidth=std_w, kernel='gaussian').fit(self.data_batch[:,4].reshape((self.batch_size,1)))
        # kde[5] = KernelDensity(bandwidth=std_w, kernel='gaussian').fit(self.data_batch[:,5].reshape((self.batch_size,1)))

        probs[0] = self.get_axis_probability(-thres_ax,thres_ax,kde[0])
        probs[1] = self.get_axis_probability(-thres_ay,thres_ay,kde[1])
        # probs[2] = self.get_axis_probability(-thres_az,thres_az,kde[2])
        # probs[3] = self.get_axis_probability(-thres_wx,thres_wx,kde[3])
        # probs[4] = self.get_axis_probability(-thres_wy,thres_wy,kde[4])
        # probs[5] = self.get_axis_probability(-thres_wz,thres_wz,kde[5])

        contact_state = (probs[0]*probs[1]).round(4) #(probs[0]*probs[1]*probs[2]*probs[3]*probs[4]*probs[5]).round(4)
        return contact_state
    

    def get_axis_probability(self,start_value, end_value, kd):
       # Number of evaluation points 
        N = self.eval_samples                                      
        step = (end_value - start_value) / (N - 1)  # Step size

        x = np.linspace(start_value, end_value, N)[:, np.newaxis]  # Generate values in the range
        kd_vals = np.exp(kd.score_samples(x))  # Get PDF values for each x
        probability = np.sum(kd_vals * step)   # Approximate the integral of the PDF

        return probability.round(4)





if __name__ == "__main__":
    topic = "/imu_rl"

    batch_size = desired_batch_size
    eval_samples = 100

    pce(topic,batch_size,eval_samples)
