# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import math
import numpy as np
from CubicSpline import *
import matplotlib.pyplot as plt
from Path_GEN import *

TARGET_U = 2 # m/s
TARGET_V = 1 # m/s
TARGET_R = 0.5 # rad/s

DT = 0.01# s

# target_sp

# ----------获得pdt------------
# pdt ---- desired postion
def get_straight_course2(dl):
    # ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    # ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    ax = [0.2,1.0,3.0,10.0,15.0,20.0,25.0,30.0,35.0]
    ay = [0.4,1.0,5.0,5.0,5.0,4.0,3.0,2.0,0.0]
    goal = [ax[-1],ay[-1]]
    cx, cy, cyaw, ck, s = calc_spline_course(
        ax, ay, ds=dl)
    return cx, cy, cyaw, ck, goal

def get_target_speed(cx,cy,cyaw):
    t_u = [0.0]*len(cx)
    t_v = [0.0]*len(cy)
    t_r = [0.0]*len(cyaw)

    for t in range(1,len(cx)):
        t_u[t] = (np.cos(cyaw[t]) * (cx[t] - cx[t - 1]) + np.sin(cyaw[t]) * (cy[t] - cy[t - 1])) / DT
        t_v[t] = (-np.sin(cyaw[t]) * (cx[t] - cx[t - 1]) + np.cos(cyaw[t]) * (cy[t] - cy[t - 1])) / DT
        t_r[t] = (cyaw[t] - cyaw[t - 1]) / DT

    return t_u,t_v,t_r

def rotate_matrix(psi):
    """全局->局部"""
    R = np.zeros((3,3))
    R[0,0] = np.cos(psi)
    R[0,1] = -1*np.sin(psi)
    R[1,0] = np.sin(psi)
    R[1,1] = np.cos(psi)
    R[2,2] = 1
    Rotate_matrix = np.linalg.inv(R)
    return R,Rotate_matrix

def pid_sim(cx,cy,cyaw,kp,ki,kd):
    Kp = kp
    Ki = ki
    Kd = kd

    # 初始化
    x = [0.0] * len(cx)
    y = [0.0] * len(cx)
    yaw = [0.0] * len(cx)

    dx = [0.0]*len(cx)
    dy = [0.0]*len(cx)
    dyaw = [0.0]*len(cx)

    # 初速度
    u = [0.0]*len(cx)
    v = [0.0]*len(cx)
    r = [0.0]*len(cx)

    t_u = [0.0]*len(cx)
    t_v = [0.0]*len(cx)
    t_r = [0.0]*len(cx)

    x_e = [0.0]*len(cx)
    y_e = [0.0]*len(cx)
    yaw_e = [0.0]*len(cx)

    du = 0
    dv = 0
    dr = 0

    ei_x = 0
    ei_y = 0
    ei_yaw = 0

    time = 0
    totaltime = [0.0]*len(cx)

    for t in range(0,len(cx)-1):

        dx[t] = cx[t] - x[t]
        dy[t] = cy[t] - y[t]
        dyaw[t] = cyaw[t] - yaw[t]

        x_e[t] = np.cos(yaw[t])*dx[t]+np.sin(yaw[t])*dy[t]
        y_e[t] = -np.sin(yaw[t])*dx[t]+np.cos(yaw[t])*dy[t]
        yaw_e[t] = dyaw[t]

        ei_x += Ki[0] * (x_e[t])
        ei_y += Ki[1] * (y_e[t])
        ei_yaw += Ki[2] * (yaw_e[t])

        if t>1:
        # 位置式PID
        #     u[t] = Kp[0] * x_e[t] + Kd[0] * (x_e[t] - x_e[t-1]) + ei_x
        #     v[t] = Kp[1] * y_e[t] + Kd[1] * (y_e[t] - y_e[t-1]) + ei_y
        #     r[t] = Kp[2] * yaw_e[t] + Kd[2] * (yaw_e[t] - yaw_e[t-1]) + ei_yaw
        # else:
        #     u[t] = Kp[0] * x_e[t] + Kd[0] * (x_e[t] ) + ei_x
        #     v[t] = Kp[1] * y_e[t] + Kd[1] * (y_e[t] ) + ei_y
        #     r[t] = Kp[2] * yaw_e[t] + Kd[2] * (yaw_e[t] ) + ei_yaw

        # 增量式PID
            u[t] = u[t-1]+ Kp[0] * (x_e[t]-x_e[t-1]) + Kd[0]*(x_e[t]-2*x_e[t-1]+x_e[t-2])+Ki[0]*x_e[t]
            v[t] = v[t-1]+ Kp[1] * (y_e[t]-y_e[t-1]) + Kd[1]*(y_e[t]-2*y_e[t-1]+y_e[t-2])+Ki[1]*y_e[t]
            r[t] = r[t-1]+ Kp[2] * (yaw_e[t]-yaw_e[t-1]) + Kd[2]*(yaw_e[t]-2*yaw_e[t-1]+yaw_e[t-2])+Ki[2]*yaw_e[t]
        elif t == 0:
            u[t] = Kp[0] * x_e[t] + Kd[0] * (x_e[t]) + Ki[0]*x_e[t]
            v[t] = Kp[1] * y_e[t] + Kd[1] * (y_e[t]) + Ki[1]*y_e[t]
            r[t] = Kp[2] * yaw_e[t] + Kd[2] * (yaw_e[t]) + Ki[2]*yaw_e[t]
        elif t == 1:
            u[t] = u[t-1]+ Kp[0] * (x_e[t]-x_e[t-1]) + Kd[0]*(x_e[t]-x_e[t-1])+Ki[0]*x_e[t]
            v[t] = v[t-1]+ Kp[1] * (y_e[t]-y_e[t-1]) + Kd[1]*(y_e[t]-y_e[t-1])+Ki[1]*y_e[t]
            r[t] = r[t-1]+ Kp[2] * (yaw_e[t]-yaw_e[t-1]) + Kd[2]*(yaw_e[t]-yaw_e[t-1])+Ki[2]*yaw_e[t]

        # update ... environment
        x[t+1] = x[t] + (np.cos(yaw[t])*u[t] - np.sin(yaw[t])*v[t]) * DT
        y[t+1] = y[t] + (np.sin(yaw[t])*u[t] + np.cos(yaw[t])*v[t]) * DT
        yaw[t+1] = yaw[t] + r[t]*DT

        time += DT
        totaltime[t] = time



    return x,y,yaw,totaltime

if __name__ == '__main__':

    dl = 1
    # cx,cy,cyaw,ck,goal = get_straight_course2(dl)
    cx,cy,cyaw,ck,goal = get_switch_back_course(dl)
    t_u,t_v,t_r = get_target_speed(cx,cy,cyaw)

    kp = [30,30,20]
    ki = [1,1,1]
    kd = [10,10,30]
    x,y,yaw,time = pid_sim(cx,cy,cyaw,kp,ki,kd)

    print(x[1])
    print(y[1])
    print(yaw[1])

    plt.subplots()
    plt.plot(cx,cy,'r-',label="desired_pos")
    plt.plot(x,y,label='pos')
    plt.legend()
    plt.grid(True)

    plt.subplots()
    plt.plot(cx,'-r',label='desired_x')
    plt.plot(x,'-g',label='x')
    plt.legend()
    plt.grid(True)

    plt.subplots()
    plt.plot(cy,'-r',label='desired_y')
    plt.plot(y,'-g',label='y')
    plt.legend()
    plt.grid(True)

    plt.subplots()
    plt.plot(cyaw,'-r',label='desired_yaw')
    plt.plot(yaw,'-g',label='yaw')
    plt.legend()
    plt.grid(True)


    plt.show()