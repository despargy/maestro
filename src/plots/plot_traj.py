import pandas as pd
import matplotlib.pyplot as plt

from mpl_toolkits import mplot3d
import numpy as np



# Read data from the CSV file
data = pd.read_csv('../bezier.csv', header=None, names=['x', 'y', 'z'])
t0_swing = 1.2
t0_super= 1.0
freq = 1.0
t_half_swing = 1.7 #(1/freq)/2+t0_swing 
dt = 0.002

t = np.arange(0,501*dt,dt)
print(len(t))
print(data['z'].shape)


plt.figure()
plt.plot(t, data['z'], label='time-Z')
plt.xlabel('Time')
plt.ylabel('Z')
plt.title('Time, t-Z of tip')
plt.legend()

plt.figure()
plt.plot(t, data['x'], label='time-X')
# plt.plot(data['t'], data['tip_z'], label='time-Z')
plt.xlabel('Time')
plt.title('Time, X-Z of tip')
plt.legend()


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(t, data['x'],  data['z'], 'gray')
ax.set_xlabel('time')
ax.set_ylabel('x')
ax.set_zlabel('z')
plt.title("Swing tip pos World")




# ### from here
# plt.figure()
# plt.plot(data['t'], data['s'], label='dis-G')

# plt.axvline(t0_super, color='g',label='t0_super')

# plt.axvline(t0_swing, color='y',label='start of swing')
# plt.axvline(t0_swing+ 1/freq, color='b',label='end of swing')

# # plt.axvline(t_half_swing, color='r')
# plt.axvline(2*t_half_swing - t0_super, color='g',label='2*t_half_swing - t0_super')
# plt.axvline(2*t_half_swing, color='k',label='2*t_half_swing')
# plt.xlabel('Time')
# plt.ylabel('X')
# plt.title('Super Gaus')
# plt.legend()



plt.show()
