# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

"""
从一点移到另一点，且：pose方向应与goal的方向一致
rho: 是机器人与目标位置之间的距离
alpha:是相对于机器人航向的目标角度
beta:是机器人位置和目标位置之间的角度加上目标角度
"""

import matplotlib.pyplot as plt
import numpy as np
from random import random

# 仿真参数
"""
Kp_rho*rho与Kp_alpha*alpha:沿着目标线驱动机器人
Kp_beta*beta:旋转线，使其平行于goal的角度
"""
Kp_rho = 3
Kp_alpha = 15
Kp_beta = -3
dt = 0.01

show_animation = True

def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot(x_traj, y_traj, 'b--')

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)

def transformation_matrix(x, y, theta):
    """位置变换矩阵"""
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

def move_to_pose(x_start,y_start,theta_start,x_goal,y_goal,theta_goal):
    """
    移动到某一位置，且方向一致
    :param x_start:robot起始x_pos
    :param y_start:robot起始y_pos
    :param theta_start:起始方向角
    :param x_goal: goal的x_pos
    :param y_goal:  goal的y_pos
    :param theta_goal: goal的方向角
    :return: XX
    """
    x = x_start
    y = y_start
    theta = theta_start

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj,y_traj = [],[]
    acc_alpha = []

    rho = np.sqrt(x_diff**2+y_diff**2)
    while rho > 0.01:

        x_traj.append(x)
        y_traj.append(y)

        x_diff = x_goal-x
        y_diff = y_goal-y

        # 将Alpha和Beta（角度差异）限制在范围内
        # [-pi~pi]避免不稳定的行为
        # 从0 rad ~ 2pi rad  只有轻微转弯（保证真实性)
        rho = np.sqrt(x_diff**2+y_diff**2)
        alpha = (np.arctan2(y_diff,x_diff)-theta+np.pi) % (2*np.pi) - np.pi
        beta = (theta_goal-theta-alpha+np.pi)%(2*np.pi) - np.pi

        acc_alpha.append(alpha)

        v = Kp_rho*rho # speed
        # w = Kp_alpha*alpha + Kp_beta*beta # rotate speed
        w = Kp_alpha*alpha

        if alpha>np.pi/2 or alpha<-np.pi/2 :
            v = -v #反向

        # update
        theta = theta + w*dt
        x = x+v*np.cos(theta)*dt e
        y = y+v*np.sin(theta)*dt



        if show_animation:
            plt.cla()
            plt.arrow(x_start,y_start,np.cos(theta_start),np.sin(theta_start),color='r',width=0.1)
            plt.arrow(x_goal,y_goal,np.cos(theta_goal),np.sin(theta_goal),color='g',width=0.1)
            plot_vehicle(x,y,theta,x_traj,y_traj)

    return acc_alpha

def main():
    # for i in range(5):
    x_start = 1
    y_start = 1
    theta_start = 0

    x_goal = 10
    y_goal = 10
    theta_goal = np.pi/2
    print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
          (x_start, y_start, theta_start))
    print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
          (x_goal, y_goal, theta_goal))
    alpha = move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)


    for i in range(len(alpha)):
        alpha[i] = np.rad2deg(alpha[i])

    plt.subplots()
    plt.plot(alpha)
    plt.show()

if __name__ =='__main__':
    main()

