import pandas as pd
import matplotlib.pyplot as plt

from mpl_toolkits import mplot3d
import numpy as np



# Read data from the CSV file
data = pd.read_csv('../data.csv', header=None, names=['t', 'y', 'tip_x', 'tip_z','s','d'])
t0_swing = 0.6
t0_super=0.25
freq = 0.5
t_half_swing = 1.6 #(1/freq)/2+t0_swing 


### from here
plt.figure()
plt.plot(data['t'], data['s'],linewidth=2)
plt.axvline(t0_super, color='g',label='t0',linewidth=2,linestyle='--')

# plt.axvline(t_half_swing, color='r')
plt.axvline(t_half_swing, color='k',label='t_half',linewidth=2,linestyle='--')
plt.xlabel('Time')
plt.ylabel('G')
plt.title('Super Gaussian')
plt.legend()

plt.show()
