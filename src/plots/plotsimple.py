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
# p_c position
p_c = data[:,3:6]
# p_d orientation
p_d = data[:,6:9]



plt.figure()
plt.plot(t_real,p_c, label="actual")
plt.plot(t_real,p_d, label="desired")
plt.xlabel("t_real")
plt.ylabel("position")
plt.legend()
plt.title("desired - actual")



plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')

