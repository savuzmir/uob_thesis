#!/usr/bin/python

import collections
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from matplotlib.lines import Line2D
import matplotlib.animation as animation

# make this a bit more serious
def prepare_data():
    with open("Trajectory.txt") as f:
        data = f.read()
    data = data.split('\n')
    return data

data = prepare_data()

def split_data(data, it, type):
    l = []
    for i in range(1, len(data) - 1):
        l.append(data[i][:].split()[it])
    if type == 'str':
        for i in range(0, len(l)):
            l[i] = str(l[i])
    elif type == 'int':
        for i in range(0, len(l)):
            l[i] = int(l[i])
    elif type == 'float':
        for i in range(0, len(l)):
            l[i] = float(l[i])
    return l

fl = 'float'
st = 'str'
intt = 'int'

x = split_data(data, 0, fl)
y = split_data(data, 1, fl)
theta = split_data(data, 2, fl)
xDot = split_data(data, 3, fl)
yDot = split_data(data, 4, fl)
wayp_num = split_data(data, 5, intt)
traj_names = split_data(data, 6, st)
traj_names = list(set(traj_names))
# Final individual trajectory names
traj_names.sort()

traj_info = collections.defaultdict(list)
traj_identifiers = []
counter = 0
wayp_num = list(set(wayp_num))

## we can probably plot directly in the loop, doesn't make much sense to store it first and then plot
for traj in traj_names:
    for el in wayp_num:
        curr_traj = str(traj) + str(el)
        traj_identifiers.append(curr_traj)
        # This assumes we have the same length of waypoints. an easy cheat would be to fill up all of them to the
        # Same length with 0s and then delete those elements from the trajectory
        try:
            traj_info[curr_traj].extend([x[counter], y[counter], theta[counter], xDot[counter], yDot[counter]])
        except:
            print("On {s} there was an error likely due to empty waypoints".format(curr_traj))
        counter += 1
    counter = 0

fig = plt.figure()
ax1 = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 2, 3)
ax3 = fig.add_subplot(2, 2, 4)

traj_coords = [()]
traj_co = 0
co = 0

plt.subplot(211)
for traj in traj_names:
    for st in traj_identifiers:
        if traj == st[0]:
            traj_coords.append((x[co], y[co], theta[co]))
            co += 1

    x_coords = [x[0] for x in traj_coords[1:]]
    y_coords = [y[1] for y in traj_coords[1:]]
    theta_info = [th[2] for th in traj_coords[1:]]
    anim_traj, = plt.plot(x_coords, y_coords, marker='o', color='Gray')
    traj_coords = [()]
    traj_co += 1

complete_traj = ()

# we want the trajectories to be pre-plotted - done
# we want the waypoint indicator - done
# we want the system state information
# we want user input
# we want optimal input


global currx, curry
currx, curry = [], []

def update(frame):
    print(frame)
    global currx, curry
    plt.subplot(211) # x coords will be replaced by the system information
    line, = plt.plot(x_coords[frame:frame+1], y_coords[frame:frame+1], marker='o', color="Black")
    plt.subplot(212)
    currx.append(x_coords[frame:frame+1][0])
    curry.append(y_coords[frame:frame+1][0])
    line2, = plt.plot(currx, curry, marker='o', color="#7EC0EE")
    line2.axes.axis([0, 1000, 0, 250])
    return line, line2

animation = animation.FuncAnimation(fig, update, interval=100, blit=True)


plt.show()