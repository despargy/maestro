#!/usr/bin/env python3
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
# Import math Library
import math 

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

# error orientation
w0 = data[:,9]
w1 = data[:,11]
w2 = data[:,13]
w3 = data[:,15]

p_c = data[:,17:20]

plt.figure()
plt.plot(t_real,p_c, label="actual")
plt.xlabel("t_real")
plt.ylabel("position")
plt.legend()
plt.title("desired - actual")


prob_0 = data[:,20]
prob_1 = data[:,21]
prob_2 = data[:,22]
prob_3 = data[:,23]


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


plt.figure()
plt.plot(t_real,w0, label="FR")
plt.plot(t_real,w1, label="FL")
plt.plot(t_real,w2, label="RR")
plt.plot(t_real,w3, label="RL")
plt.xlabel("t_real")
plt.ylabel("Weights")
plt.legend()
plt.title("Weights")


leg_0 = data[:,24:27] #x,y,z
leg_1 = data[:,27:30] #x,y,z
leg_2 = data[:,30:33] #x,y,z
leg_3 = data[:,33:36] #x,y,z

plt.figure()
plt.plot(t_real,leg_0, label="FR")
plt.plot(t_real,leg_1, label="FL")
plt.plot(t_real,leg_2, label="RR")
plt.plot(t_real,leg_3, label="RL")
plt.xlabel("t_real")
plt.ylabel("Tip pos")
plt.legend()
plt.title("Tip pos world frame")

plt.figure()
plt.plot(t_real,leg_0[:,0], label="FR")
plt.plot(t_real,leg_1[:,0], label="FL")
plt.plot(t_real,leg_2[:,0], label="RR")
plt.plot(t_real,leg_3[:,0], label="RL")
plt.xlabel("t_real")
plt.ylabel("Tip pos X")
plt.legend()
plt.title("Tip pos X world frame")

plt.figure()
plt.plot(t_real,leg_0[:,1], label="FR")
plt.plot(t_real,leg_1[:,1], label="FL")
plt.plot(t_real,leg_2[:,1], label="RR")
plt.plot(t_real,leg_3[:,1], label="RL")
plt.xlabel("t_real")
plt.ylabel("Tip pos Y")
plt.legend()
plt.title("Tip pos Y world frame")

plt.figure()
plt.plot(t_real,leg_0[:,2], label="FR")
plt.plot(t_real,leg_1[:,2], label="FL")
plt.plot(t_real,leg_2[:,2], label="RR")
plt.plot(t_real,leg_3[:,2], label="RL")
plt.xlabel("t_real")
plt.ylabel("Tip pos Z")
plt.legend()
plt.title("Tip pos Z world frame")

plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')

