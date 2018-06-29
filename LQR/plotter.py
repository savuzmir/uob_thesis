#!/usr/bin/python

import collections
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from matplotlib.lines import Line2D
import matplotlib.animation as animation



with open("trajectory.txt") as f:
    data = f.read()

data = data.split('\n')


coordinates = np.empty((5, 6,))

# Vector with trajectory names
traj_names = [str(data[i][:].split()[6]) for i in range(1, len(data) - 1)]


num = 0
for curr, next in zip(traj_names, traj_names[1:]):
    if curr==next:
        num += 1
    else:
        num = 0

traj_names = list(set(traj_names))
traj_names.sort()


# Vector with trajectory waypoint numbers
wayp_num = [int(data[i][:].split()[5]) for i in range(1, len(data) - 1)]

temp_traj_info = collections.defaultdict(list)
traj_info = collections.defaultdict(list)

x = [float(data[i][:].split()[0]) for i in range(1, len(data) - 1)]
y = [float(data[i][:].split()[1]) for i in range(1, len(data) - 1)]
theta = [float(data[i][:].split()[2]) for i in range(1, len(data) - 1)]

xDot = [float(data[i][:].split()[3]) for i in range(1, len(data) - 1)]
yDot = [float(data[i][:].split()[4]) for i in range(1, len(data) - 1)]


counter = 0

fig = plt.figure()
ax1 = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 2, 3)
ax3 = fig.add_subplot(2, 2, 4, projection='3d')

traj_identifiers = []

wayp_num = list(set(wayp_num))

## we can probably plot directly in the loop, doesn't make much sense to store it first and then plot
for traj in traj_names:
    for el in wayp_num:
        curr_traj = str(traj) + str(el)
        traj_identifiers.append(curr_traj)
        # This assumes we have the same length of waypoints. an easy cheat would be to fill up all of them to the
        # Same length with 0s and then delete those elements from the trajectory
        temp_traj_info[curr_traj].extend([x[counter], y[counter], theta[counter], xDot[counter], yDot[counter]])
        counter += 1
    counter = 0


for st in traj_identifiers:
    traj_info[st] = temp_traj_info[st]

traj_coords = [()]
traj_co = 0
co = 0

# Set individual trajectory colours this is provisional because it needs to be able to work
# For any given amount of trajectories will once there's more of them

colour_list = ["#952B14", "#BB3619", "#7C2411", "#5F1C0E", "#47150B", "#F24623"]


for traj in traj_names:
    for st in traj_identifiers:
        if traj == st[0]:

            traj_coords.append((x[co], y[co], theta[co]))
            co += 1


    x_coords = [x[0] for x in traj_coords[1:]]
    y_coords = [y[0] for y in traj_coords[1:]]
    theta_info = [th[0] for th in traj_coords[1:]]
    plt.plot(x_coords, y_coords, theta_info, marker='o', color=colour_list[traj_co])
    traj_coords = [()]
    x_coords = []
    y_coords = []
    traj_co += 1
#    co = 0

plt.show()

complete_traj = ()

# We want an indicator in the animation of which is the current waypoint.
# We want an indicator in the animation of the user input in a separate plot (e.g. up down left, right)
    # this  will be a trajectory point that will be voted to be the nearest neighbour at each point so our
    # function should be returning this as well
# We want information of where the system is at a given time - this will
# And we want information of where the lqr is pulling the user towards

class SubplotAnimation(animation.TimedAnimation):
    def __init__(self, trajectory_info, fig, ax1, ax2, ax3):


        self.t = np.linspace(0, 80, 400)
        self.x = np.cos(2 * np.pi * self.t / 10.)
        self.y = np.sin(2 * np.pi * self.t / 10.)
        self.z = 10 * self.t

        ax1.set_xlabel('x')
        ax1.set_ylabel('y')
        self.line1 = Line2D([], [], color='black')
        self.line1a = Line2D([], [], color='red', linewidth=2)
        self.line1e = Line2D(
            [], [], color='red', marker='o', markeredgecolor='r')
        ax1.add_line(self.line1)
        ax1.add_line(self.line1a)
        ax1.add_line(self.line1e)
        ax1.set_xlim(-1, 1)
        ax1.set_ylim(-2, 2)
        ax1.set_aspect('equal', 'datalim')

        ax2.set_xlabel('y')
        ax2.set_ylabel('z')
        self.line2 = Line2D([], [], color='black')
        self.line2a = Line2D([], [], color='red', linewidth=2)
        self.line2e = Line2D(
            [], [], color='red', marker='o', markeredgecolor='r')
        ax2.add_line(self.line2)
        ax2.add_line(self.line2a)
        ax2.add_line(self.line2e)
        ax2.set_xlim(-1, 1)
        ax2.set_ylim(0, 800)

        ax3.set_xlabel('x')
        ax3.set_ylabel('z')
        self.line3 = Line2D([], [], color='black')
        self.line3a = Line2D([], [], color='red', linewidth=2)
        self.line3e = Line2D(
            [], [], color='red', marker='o', markeredgecolor='r')
        ax3.add_line(self.line3)
        ax3.add_line(self.line3a)
        ax3.add_line(self.line3e)
        ax3.set_xlim(-1, 1)
        ax3.set_ylim(0, 800)

        animation.TimedAnimation.__init__(self, fig, interval=50, blit=True)

    def _draw_frame(self, framedata):
        i = framedata
        head = i - 1
        head_slice = (self.t > self.t[i] - 1.0) & (self.t < self.t[i])

        self.line1.set_data(self.x[:i], self.y[:i])
        self.line1a.set_data(self.x[head_slice], self.y[head_slice])
        self.line1e.set_data(self.x[head], self.y[head])

        self.line2.set_data(self.y[:i], self.z[:i])
        self.line2a.set_data(self.y[head_slice], self.z[head_slice])
        self.line2e.set_data(self.y[head], self.z[head])

        self.line3.set_data(self.x[:i], self.z[:i])
        self.line3a.set_data(self.x[head_slice], self.z[head_slice])
        self.line3e.set_data(self.x[head], self.z[head])

        self._drawn_artists = [self.line1, self.line1a, self.line1e,
                               self.line2, self.line2a, self.line2e,
                               self.line3, self.line3a, self.line3e]

    def new_frame_seq(self):
        return iter(range(self.t.size))

    def _init_draw(self):
        lines = [self.line1, self.line1a, self.line1e,
                 self.line2, self.line2a, self.line2e,
                 self.line3, self.line3a, self.line3e]
        for l in lines:
            l.set_data([], [])

ani = SubplotAnimation(traj_identifiers, fig, ax1, ax2, ax3)
# ani.save('test_sub.mp4')
plt.show()

