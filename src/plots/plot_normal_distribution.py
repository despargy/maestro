import pandas as pd
import matplotlib.pyplot as plt

from mpl_toolkits import mplot3d
import numpy as np



# Read data from the CSV file
data = pd.read_csv('../data.csv', header=None, names=['t', 'y', 'tip_x', 'tip_z','s','d'])
t0_swing = 0.25
t0_super=0.25
freq = 0.5
t_half_swing = (1/freq)/2+t0_swing 

# Plot the normal distribution function
# plt.figure()
# plt.plot(data['t'], data['y'], label='Normal Distribution')
# plt.xlabel('Time (t)')
# plt.ylabel('Probability Density')
# plt.title('Normal Distribution Function')
# plt.legend()
# plt.grid()

# plt.figure()
# plt.plot(data['tip_x'], data['tip_z'], label='X-Z POS')
# plt.xlabel('X')
# plt.ylabel('Z')
# plt.title('X-Z 2D plot of tip')
# plt.legend()

# plt.figure()
# plt.plot(data['t'], data['tip_x'], label='time-X')
# # plt.plot(data['t'], data['tip_z'], label='time-Z')
# plt.xlabel('Time')
# plt.title('Time, X-Z of tip')
# plt.legend()

### from here
plt.figure()
plt.plot(data['t'], data['s'], label='dis-G')
plt.axvline(t0_super, color='r',label='t0_super/swing')
# plt.axvline(0.5, color='y',label='t0_swing')
plt.axvline(t_half_swing, color='r')
plt.axvline(2*t_half_swing - t0_super, color='g',label='2*t_half_swing - t0_super')
plt.axvline(2*t_half_swing, color='k',label='2*t_half_swing')
plt.xlabel('Time')
plt.ylabel('X')
plt.title('Super Gaus')
plt.legend()

# plt.figure()
# plt.plot(data['t'], data['tip_x'], label='t-x')
# plt.axvline(0.25, color='b',label='t0_swing')
# # plt.axvline(4, color='b',label='1/freq')
# plt.axvline(4+0.25, color='b',label='1/freq+t0_swing')
# plt.xlabel('Time')
# plt.ylabel('X')
# plt.title('Super Gaus')
# plt.legend()

# plt.figure()
plt.plot(data['t'], data['tip_z'], label='t-z')
# plt.axvline(0.25, color='b',label='t0_swing')
# plt.axvline(2+0.25, color='b',label='1/freq+t0_swing')
# plt.xlabel('Time')
# plt.ylabel('X')
# plt.title('Super Gaus')
# plt.legend()

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.plot3D(data['t'], data['tip_x'], data['tip_z'], 'gray')


### to here


plt.show()