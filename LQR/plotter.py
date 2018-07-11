#!/usr/bin/python
import collections
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import time
from matplotlib.lines import Line2D
import matplotlib.animation as animation

# We will need these two functions multiple times
def prepare_data(file):
    with open(file) as f:
        data = f.read()
    data = data.split('\n')
    return data

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

################################
####### Prepare the data #######
################################

data = prepare_data('Trajectory.txt')
x_hist = prepare_data('StateHistory.txt')
nn_hist = prepare_data('UserHistory.txt')
u_hist_star = prepare_data('OptimalInputHistory.txt')
u_hist = prepare_data('InputHistory.txt')

fl = 'float'
st = 'str'
nr = 'int'

################################
##### Define all variables #####
################################

# Trajectory variables
x = split_data(data, 0, fl)
y = split_data(data, 1, fl)
theta = split_data(data, 2, fl)
xDot = split_data(data, 3, fl)
yDot = split_data(data, 4, fl)
wayp_num = split_data(data, 5, nr)
traj_names = split_data(data, 6, st)
traj_names = list(set(traj_names))

# Final individual trajectory names
traj_names.sort()

# State history variables
x_hist_x = split_data(x_hist, 0, fl)
x_hist_y = split_data(x_hist, 1, fl)
x_hist_theta = split_data(x_hist, 2, fl)

# Nearest neighbour trajectory history
nn_hist = split_data(nn_hist, 0, st)

# User input history
u_hist_xDot = split_data(u_hist, 0, fl)
u_hist_yDot = split_data(u_hist, 1, fl)

# Optimal input history
u_hist_xDot_star = split_data(u_hist_star, 0, fl)
u_hist_yDot_star = split_data(u_hist_star, 1, fl)

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


################################
## Start plotting everything ###
################################

fig = plt.figure()
ax1 = fig.add_subplot(3, 1, 1)
ax2 = fig.add_subplot(3, 2, 3)
ax3 = fig.add_subplot(3, 2, 4)
ax4 = fig.add_subplot(3, 2, 5)
ax5 = fig.add_subplot(3, 2, 6)


traj_coords = [()]
traj_co = 0
co = 0

plt.subplot(311)
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

font = {'family': 'arial',
        'color':  'black',
        'weight': 'normal',
        'size': 11,
        }

u_x, u_y, x_x, x_y, u_opt_x, u_opt_y = [], [], [], [], [], []


def filter(input_list, frame, el):
    return input_list[frame:frame+1][el]

def update(frame):
    # This is the trajectory history
    curr_wayp = filter(nn_hist, frame, 0)
    wayp_x = traj_info[curr_wayp][0]
    wayp_y = traj_info[curr_wayp][1]

    # This is the user input history
    user_x = filter(u_hist_xDot, frame, 0)
    user_y = filter(u_hist_yDot, frame, 0)
    u_x.append(user_x)
    u_y.append(user_y)

    # this is the optimal input history
    u_star_x = filter(u_hist_xDot_star, frame, 0)
    u_star_y = filter(u_hist_yDot_star, frame, 0)
    u_opt_x.append(u_star_x)
    u_opt_y.append(u_star_y)

    # State history
    state_x = filter(x_hist_x, frame, 0)
    state_y = filter(x_hist_y, frame, 0)
    x_x.append(state_x)
    x_y.append(state_y)

    plt.subplot(311)
    # this plots the current waypoint
    plt.title('Potential trajectories with unfolding current state (red) and nearest waypoint (black)', fontdict=font)
    line, = plt.plot(wayp_x, wayp_y, marker='o', color="Black")

    # this plots the unfolding state
    line2, = plt.plot(x_x, x_y, marker='o', color="Red")

    plt.subplot(323)
    plt.title('User input', fontdict=font)

    # this plots the user input
    line3, = plt.plot(u_x, u_y, marker='o', color="#7EC0EE")
    line3.axes.axis([-1, 1, -1, 1])
    u_x.pop()
    u_y.pop()

    plt.subplot(324)
    plt.title('Optimal input', fontdict=font)
    # this plots the optimal input info
    line4, = plt.plot(u_opt_x, u_opt_y, marker='o', color="Green")
    line4.axes.axis([min(u_opt_x), max(u_opt_x), min(u_opt_y), max(u_opt_y)])

    # finally, we should have an obtained and optimal state value comparison.
    plt.subplot(325)
    plt.title('Unfolding (red) and predicted (gray) state', fontdict=font)

    # unfolding state
    line5, = plt.plot(x_x, x_y, marker='o', color="Red")
    # unfolding state prediction
    line6, = plt.plot(u_opt_x, u_opt_y, marker='o', color="Gray")

    plt.subplot(326)
    ax5.axis('off')
    time_text = plt.text(.05, .8, '', fontsize=10)

    plt.title('Meta-information', fontdict=font)
    time_text.set_text("Elapsed waypoint: " + str(wayp_num[frame]))


    return line, line2, line3, line4, line5, line6, time_text

plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.5,
                    wspace=0.35)

animation = animation.FuncAnimation(fig, update, interval=100, blit=True)


plt.show()