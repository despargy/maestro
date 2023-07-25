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
w0 = data[:,15]
w1 = data[:,18]
w2 = data[:,21]
w3 = data[:,24]


p_c = data[:,27:30]
p_d = data[:,30:33]

plt.figure()
plt.plot(t_real,p_c, label="actual")
plt.plot(t_real,p_d, label="desired")
plt.xlabel("t_real")
plt.ylabel("position")
plt.legend()
plt.title("desired - actual")




prob_0 = data[:,33]
prob_1 = data[:,34]
prob_2 = data[:,35]
prob_3 = data[:,36]


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


plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')

