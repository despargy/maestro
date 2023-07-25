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


# error orientation
w0 = data[:,15]
w1 = data[:,18]
w2 = data[:,21]
w3 = data[:,24]

# w0[w0 == math.inf] = 150

inf_id = np.argmax(w0)

w0_ok = w0.copy()
# print(inf_id)
w0_inf = w0.copy()
w0_inf[0:inf_id] = None
w0_inf[inf_id:] = 150

fig, ax = plt.subplots() 
circle1 = plt.Circle((t_real[inf_id], w0[inf_id-1]), 0.2, color='b', fill=False)
ax.add_patch(circle1)


math.atanh(2.0)

# plt.plot(t_real,w0_ok)
# plt.plot(t_real,w0_inf, 'k--')
plt.plot(t_real,w0)
plt.plot(t_real,w1)
plt.plot(t_real,w2)
plt.plot(t_real,w3)

plt.xlabel("t_real")
plt.ylabel("Weights")
plt.title("Weights")


# f, (ax, ax2) = plt.subplots(2, 1, sharey=True, facecolor='w')

# # plot the same data on both axes
# ax.plot(t_real, w0)

# ax2.plot(t_real, w0)
# ax2.plot(t_real, w1)
# ax2.plot(t_real, w2)
# ax2.plot(t_real, w3)


# ax.set_ylim(0, 60)
# ax2.set_ylim(1000, 10042.5)

# # hide the spines between ax and ax2
# ax.spines['bottom'].set_visible(False)
# ax2.spines['top'].set_visible(False)
# ax.xaxis.tick_top()
# ax.tick_params(labeltop='off')
# ax2.xaxis.tick_bottom()

# This looks pretty good, and was fairly painless, but you can get that
# cut-out diagonal lines look with just a bit more work. The important
# thing to know here is that in axes coordinates, which are always
# between 0-1, spine endpoints are at these locations (0, 0), (0, 1),
# (1, 0), and (1, 1).  Thus, we just need to put the diagonals in the
# appropriate corners of each of our axes, and so long as we use the
# right transform and disable clipping.

# d = .015  # how big to make the diagonal lines in axes coordinates
# # arguments to pass plot, just so we don't keep repeating them
# kwargs = dict(transform=ax.transAxes, color='k', clip_on=False)
# ax.plot((1-d, 1+d), (-d, +d), **kwargs)
# ax.plot((1-d, 1+d), (1-d, 1+d), **kwargs)

# kwargs.update(transform=ax2.transAxes)  # switch to the bottom axes
# ax2.plot((-d, +d), (1-d, 1+d), **kwargs)
# ax2.plot((-d, +d), (-d, +d), **kwargs)

# What's cool about this is that now if we vary the distance between
# ax and ax2 via f.subplots_adjust(hspace=...) or plt.subplot_tool(),
# the diagonal lines will move accordingly, and stay right at the tips
# of the spines they are 'breaking'


plt.show()
plt.waitforbuttonpress(0) # this will wait for indefinite timeplt.close(fig)
plt.close('all')

