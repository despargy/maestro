#!/usr/bin/env python3
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
# Import math Library
import math 

data = np.genfromtxt("/home/despargy/go1_ws/src/maestro/CSV/Log.csv", delimiter=",", skip_header=1) #TODO

# times
t_real = data[:,0]
tv =  data[:,1]
d_tv = data[:,2]
# error position
e_p = data[:,3:6]
e_p_norm = np.empty(shape=np.shape(t_real))
for i in range(np.shape(t_real)[0]):
    e_p_norm[i] = math.sqrt( e_p[i,0]**2 + e_p[i,1]**2 + e_p[i,2]**2 )

# error orientation
e_o = data[:,6:9]
e_o_norm = np.empty(shape=np.shape(t_real))
for i in range(np.shape(t_real)[0]):
    e_o_norm[i] = math.sqrt( e_o[i,0]**2 + e_o[i,1]**2 + e_o[i,2]**2 )

# # error linear vel
# e_v_lin = data[:,9:12]
# e_v_lin_norm = np.empty(shape=np.shape(t_real))
# for i in range(np.shape(t_real)[0]):
#     e_v_lin_norm[i] = math.sqrt( e_v_lin[i,0]**2 + e_v_lin[i,1]**2 + e_v_lin[i,2]**2 )

# # error ang vel
# e_v_ang = data[:,12:15]
# e_v_ang_norm = np.empty(shape=np.shape(t_real))
# for i in range(np.shape(t_real)[0]):
#     e_v_ang_norm[i] = math.sqrt( e_v_ang[i,0]**2 + e_v_ang[i,1]**2 + e_v_ang[i,2]**2 )


# error orientation
w0 = data[:,15]
w1 = data[:,18]
w2 = data[:,21]
w3 = data[:,24]
# w = np.empty( shape=[np.shape(t_real)[0],4])
# w[:,0] = w0
# w[:,1] = w1
# w[:,2] = w2
# w[:,3] = w3


prob_0 = data[:,33]
prob_1 = data[:,34]
prob_2 = data[:,35]
prob_3 = data[:,36]

plt.figure()
plt.plot(t_real,e_p_norm)
plt.xlabel("t_real")
plt.ylabel("Error Position Norm")
plt.title("Error Position Norm")


plt.figure()
plt.plot(t_real,e_o_norm)
plt.xlabel("t_real")
plt.ylabel("Error Orientation Norm")
plt.title("Error Orientation Norm")

# plt.figure()
# plt.plot(t_real,e_v_lin_norm)
# plt.xlabel("t_real")
# plt.ylabel("Error Linear Velocity Norm")

# plt.figure()
# plt.plot(t_real,e_v_ang_norm)
# plt.xlabel("t_real")
# plt.ylabel("Error Angular Velocity Norm")

plt.figure()
plt.plot(t_real,w0)
plt.plot(t_real,w1)
plt.plot(t_real,w2)
plt.plot(t_real,w3)
plt.xlabel("t_real")
plt.ylabel("Weights")
plt.title("Weights")


plt.figure()
plt.plot(t_real,d_tv)
plt.xlabel("t_real")
plt.ylabel("Derivative of Virtual time")
plt.title("Derivative of Virtual time")

plt.figure()
plt.plot(t_real,tv)
plt.xlabel("t_real")
plt.ylabel("Virtual time")
plt.title("Virtual time")



plt.figure()
plt.plot(t_real,prob_0)
plt.plot(t_real,prob_1)
plt.plot(t_real,prob_2)
plt.plot(t_real,prob_3)
plt.xlabel("t_real")
plt.ylabel("Probs")
plt.title("Probs ALL")

plt.figure()
plt.plot(t_real,prob_0)
plt.xlabel("t_real")
plt.ylabel("Prob 0")
plt.title("Probs O")


plt.figure()
plt.plot(t_real,prob_1)
plt.xlabel("t_real")
plt.ylabel("Prob 1")
plt.title("Probs 1")


plt.figure()
plt.plot(t_real,prob_2)
plt.xlabel("t_real")
plt.ylabel("Prob 2")
plt.title("Probs 2")


plt.figure()
plt.plot(t_real,prob_3)
plt.xlabel("t_real")
plt.ylabel("Prob 3")
plt.title("Probs 3")

plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')

# plt.plot(df.Name, df.Marks)
# plt.show()
# def plotResults():
        
#         fig, axes = plt.subplots(3)
#         fig.suptitle('Vertically stacked subplots')

#         axes[0].plot(self.timestamp, self.x, "tab:red", label="X")
#         axes[0].plot(self.timestamp, self.y, "tab:green", label="Y")
#         axes[0].plot(self.timestamp, self.z, "tab:blue", label="Z")
#         axes[0].set_title("Position")
#         axes[0].set_ylabel("Degrees/s or time?")
#         axes[0].grid()
#         axes[0].legend()


#         axes[1].plot(self.timestamp, self.dx, "tab:red", label="X")
#         axes[1].plot(self.timestamp, self.dy, "tab:green", label="Y")
#         axes[1].plot(self.timestamp, self.dz, "tab:blue", label="Z")
#         axes[1].set_title("Velocity")
#         axes[1].set_ylabel("Degrees/s or time?")
#         axes[1].grid()
#         axes[1].legend()

        
#         axes[2].plot(self.timestamp, self.roll_low, "tab:red", label="X")
#         axes[2].plot(self.timestamp, self.pitch_low, "tab:green", label="Y")
#         axes[2].plot(self.timestamp, self.yaw_low, "tab:blue", label="Z")
#         axes[2].set_title("RPY low")
#         axes[2].set_ylabel("Degrees/s or time?")
#         axes[2].grid()
#         axes[2].legend()


#         plt.show()