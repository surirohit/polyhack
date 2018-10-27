# Algorithm to compute potential field

import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import sqrt

q_goalx = 2.6 # m
q_goaly = 2.6 # m

obstacles_x = np.array([2.0, 1.5, 3.15, 3.0])
obstacles_y = np.array([2.8, 1.0, 0.7, 2.0])

drone_x = 0.5
drone_y = 0.5

def calc_potential_field(obstacles_x, obstacles_y, drone_x, drone_y, q_goalx, q_goaly):
    # Parameters
    K_att = 5.0  # attractive potential gain
    K_rep = 100.0  # repulsive potential gain
    d = 0.1 # m, tolerance to reach goal
    rho_naut = 0.6 # m, distance of influence

    # Attractive potential
    q = np.array([drone_x, drone_y])

    q_goal = np.array([q_goalx, q_goaly])

    rho_goal = sqrt((q[0]-q_goal[0])**2 + (q[1]-q_goal[1])**2)

    U_att = 0.5 * K_att * rho_goal**2

    # The Gradient grad_rhogoal(q)
    grad_rhogoal = (q - q_goal)/rho_goal

    grad_U = -0.5 * K_att *(2 * rho_goal) * grad_rhogoal

    F_att = -grad_U
    # F_att is a vector directed toward q_goal with magnitude linearly related to the distance from q to q_goal
    # F_att converges linearly to zero as q approaches q_goal - good for stability
    # F_att grows without bound as q moves away from q_goal - not so good

    # Conic Well Attractive Potential
        # Idea: Use a `conic well' to keep F_att bounded
    if rho_goal <= d:
        U_att = 0.5 * K_att * rho_goal**2
        F_att = - K_att*(q - q_goal)
    else:
        U_att = d * K_att * rho_goal
        F_att = - d * K_att * (q - q_goal)/rho_goal

    # The Repulsive Potential
        # Idea: A should be repelled from obstacles
            # Never want to let A `hit` and obstacle
            # If A is far from obstacle, don't want obstacle to affect A's motion
        # One Choice for U_rep

    rho = 10**6
    for i in range(0,len(obstacles_x)):
        rho_test = sqrt((q[0] - obstacles_x[i])**2 + (q[1] - obstacles_y[i])**2)
        if rho_test < rho:
            rho = rho_test
            q_c = [obstacles_x[i], obstacles_y[i]]

    if rho <= rho_naut:
        U_rep = 0.5 * K_rep * (1/rho - 1/rho_naut)
        mag_q_minus_qc = sqrt((q[0] - q_c[0])**2 + (q[1] - q_c[1])**2)
        F_rep = K_rep * (1/rho - 1/rho_naut) * (1/rho**2) * (q - q_c)/mag_q_minus_qc
    else:
        U_rep = 0
        F_rep = 0

    # Gradient Descent Planning
    U = U_att + U_rep
    F = F_att + F_rep

    # 1. Let q_0 = q_init, i = 0

    delta = 0.01*sqrt((q[0] - q_goal[0])**2 + (q[1] - q_goal[1])**2)
    q_next = q + delta*F/(sqrt(F[0]**2 + F[1]**2))

    return q_next

def planPath(obstacles_x, obstacles_y, drone_x, drone_y, q_goalx, q_goaly):
    tol = 0.1 # m
    # iter_max = 1000
    iter = 0
    q = np.array([drone_x, drone_y])
    q_goal = np.array([q_goalx, q_goaly])
    dist = sqrt((q[0] - q_goal[0])**2 + (q[1] - q_goal[1])**2)
    x = np.array([drone_x])
    y = np.array([drone_y])
    while (dist > tol): # and iter <= iter_max):
        q_next = calc_potential_field(obstacles_x, obstacles_y, drone_x, drone_y, q_goalx, q_goaly)
        dist = sqrt((q_next[0] - q_goal[0])**2 + (q_next[1] - q_goal[1])**2)
        iter = iter + 1
        drone_x = q_next[0]
        drone_y = q_next[1]
        print(drone_x)
        if (dist <= tol or (iter % 2)==0):
            print(drone_x)
            x = np.append(x,drone_x)
            y = np.append(y,drone_y)
        # time.sleep(0.0625)
        # print(dist)
        # print(q_next)
    return x, y

x, y = planPath(obstacles_x, obstacles_y, drone_x, drone_y, q_goalx, q_goaly)
#print(x)
#print(y)
plt.plot(x,y)
plt.show()