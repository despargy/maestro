import pandas as pd
import matplotlib.pyplot as plt

from mpl_toolkits import mplot3d
import numpy as np



# Read data from the CSV file
data = pd.read_csv('../data.csv', header=None, names=['t', 'y', 'tip_x', 'tip_z','s','d'])

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
# plt.plot(data['t'], data['tip_z'], label='time-Z')
# plt.xlabel('Time')
# plt.title('Time, X-Z of tip')
# plt.legend()


plt.figure()
plt.plot(data['t'], data['s'], label='dis-G')
plt.xlabel('Time')
plt.ylabel('X')
plt.title('Super Gaus')
plt.legend()


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(data['t'], data['tip_x'], data['tip_z'], 'gray')




plt.show()
