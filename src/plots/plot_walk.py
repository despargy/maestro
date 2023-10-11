#!/usr/bin/env python3
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
# Import math Library
import math 
from mpl_toolkits import mplot3d
import numpy as np

# data = np.genfromtxt("/home/despargy/go1_ws/src/maestro/CSV/Inf/Log-3000-350-normal.csv", delimiter=",", skip_header=1) #TODO
# data = np.genfromtxt("/home/despargy/go1_ws/src/maestro/SavedCSVfromExperiments/NewExperiments/Simulation/Log-Inf-3000-350-normal.csv", delimiter=",", skip_header=1) #TODO

data = np.genfromtxt("/home/despargy/go1_ws/src/maestro/CSV/Log.csv", delimiter=",", skip_header=1) #TODO

# times
t_real = data[:,0]
tv =  data[:,1]
d_tv = data[:,2]
# error position
e_p = data[:,3:6]

# error orientation
e_o = data[:,6:9]

p_c = data[:,9:12]
p_T = data[:,12:15]

plt.figure()
plt.plot(t_real,p_c, label="actual")
plt.plot(t_real,p_T, label="target")
plt.xlabel("t_real")
plt.ylabel("position")
plt.legend()
plt.title("target - actual")


plt.figure()
plt.plot(t_real,e_p[:,0], label="x")
plt.plot(t_real,e_p[:,1], label="y")
plt.plot(t_real,e_p[:,2], label="z")
plt.xlabel("t_real")
plt.ylabel("e_p")
plt.legend()
plt.title("e_p")



plt.figure()
plt.plot(t_real,e_o[:,0], label="x")
plt.plot(t_real,e_o[:,1], label="y")
plt.plot(t_real,e_o[:,2], label="z")
plt.xlabel("t_real")
plt.ylabel("e_o")
plt.legend()
plt.title("e_o")


d_traj_tip = data[:,15:18] #x,y,z
swing0_now = data[:,18:21] #x,y,z

plt.figure()
plt.plot(t_real,d_traj_tip, label="desired")
plt.xlabel("t_real")
plt.ylabel("Desired tip pos")
plt.legend()
plt.title("Derised trajectory tip frame")


w0 = data[:,21]
w1 = data[:,23]
w2 = data[:,25]
w3 = data[:,27]

plt.figure()
plt.plot(t_real,w0, label="FR",linewidth=2)
plt.plot(t_real,w1, label="FL",linewidth=2)
plt.plot(t_real,w2, label="RR",linewidth=2)
plt.plot(t_real,w3, label="RL",linewidth=2)
plt.xlabel("t_phase")
plt.ylabel("Weights")
plt.legend()
plt.title("Weights of x-axis")

d_traj_0frame = data[:,29:32] #x,y,z
plt.figure()
plt.plot(t_real,d_traj_0frame[:,0], label="x")
plt.plot(t_real,d_traj_0frame[:,1], label="y")
plt.plot(t_real,d_traj_0frame[:,2], label="z")

plt.xlabel("t_real")
plt.ylabel("Traj Oframe")
plt.legend()
plt.title("Traj tip Oframe")

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(t_real[0:1200], d_traj_0frame[0:1200,0], d_traj_0frame[0:1200,2], 'gray')
ax.set_xlabel('time')
ax.set_ylabel('x')
ax.set_zlabel('z')
plt.legend()
plt.title("Derised swinging leg trajectory, CoM frame")

t0_swing = 0.3

plt.figure()
plt.plot(t_real,d_traj_0frame[:,0], label="x")
plt.plot(t_real,d_traj_0frame[:,1], label="y")
plt.plot(t_real,d_traj_0frame[:,2], label="z")
plt.axvline(t0_swing, color='y',label='start of swing')

plt.xlabel("t_real")
plt.ylabel("Desired tip pos from CoM frame")
plt.legend()
plt.title("Derised trajectory CoM frame")



qout0_ = data[:,32:35] #0,1,2
plt.figure()
plt.plot(t_real,qout0_[:,0], label="q_out 0")
plt.plot(t_real,qout0_[:,1], label="q_out 1") 
plt.plot(t_real,qout0_[:,2], label="q_out 2")

# plt.plot(t_real,swing0_now, label="now")
plt.xlabel("t_real")
plt.ylabel("Swing tip q_out ")
plt.legend()
plt.title("Swing tip q_out")


tip0_ = data[:,35:38] #x,y,z

plt.figure()
plt.plot(t_real,tip0_[:,0], label="now x")
plt.plot(t_real,tip0_[:,1], label="now y")
plt.plot(t_real,tip0_[:,2], label="now z")

# plt.plot(t_real,swing0_now, label="now")
plt.xlabel("t_real")
plt.ylabel("Swing tip pos World")
plt.legend()
plt.title("Swing tip pos World")


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(t_real, tip0_[:,0], tip0_[:,2], 'gray')
ax.set_xlabel('time')
ax.set_ylabel('x')
ax.set_zlabel('z')
plt.title("Swing tip pos World")

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(t_real, tip0_[:,0], tip0_[:,1], 'gray')
ax.set_xlabel('time')
ax.set_ylabel('x')
ax.set_zlabel('y')
plt.title("Swing tip pos World")



# plt.figure()
# plt.plot(t_real,data[:,23], label="FR")
# plt.plot(t_real,data[:,26], label="FL")
# plt.plot(t_real,data[:,29], label="RR")
# plt.plot(t_real,data[:,32], label="RL")
# plt.xlabel("t_real")
# plt.ylabel("Weights z")
# plt.legend()
# plt.title("Weights z")




plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')

