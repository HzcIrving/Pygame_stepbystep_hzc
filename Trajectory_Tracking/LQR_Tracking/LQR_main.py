# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

# from LQR_Steering_Control import *
import  LQR_Steering_Control as LSC
import math
import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as la
import CubicSpline

# LQR parameter
lqr_Q = np.eye(5)
lqr_R = np.eye(2)
dt = 0.1  # time tick[s]
L = 0.5  # Wheel base of the vehicle [m]
max_steer = np.deg2rad(45.0)  # maximum steering angle[rad]

show_animation = True

# 目标
ax = [0.0, 6.0, 12.5, 10.0, 17.5, 20.0, 25.0]
ay = [0.0, -3.0, -5.0, 6.5, 3.0, 0.0, 0.0]
goal = [ax[-1], ay[-1]]   # 最后一个点
print(goal)

cx,cy,cyaw,ck,s = CubicSpline.calc_spline_course(
    ax,ay,ds=0.1
) #插值 获得插值平面

def test_cubic():
    # 测试插值效果
    plt.subplots(1)
    plt.plot(cx,cy,"-r",label='spline')
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.show()

target_speed = 10.0/3.6 # km/h  仿真参数

# -------计算速度曲线-------------
sp = LSC.calc_speed_profile(cyaw,target_speed) #开始加速，后面减速
# plt.plot(sp) # 速度曲线

# -------开始仿真----------------
t,x,y,yaw,v = LSC.do_simulation(cx,cy,cyaw,ck,sp,goal)

if show_animation:
    # 结束跟踪后的曲线...
    plt.close()
    plt.subplot(131)
    plt.plot(ax,ay,"xb",label="Waypoints") # landmark点(用于spline的点)
    plt.plot(cx,cy,"-r",label="Target Course") # 目标点
    plt.plot(x,y,"-g",label="Tracking")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    # 偏航角变化曲线
    plt.subplot(132)
    # s: line length 走过的路程
    # iyaw : 偏航角大小
    plt.plot(s,[np.rad2deg(iyaw) for iyaw in cyaw],"-r",label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    # 曲率曲线
    plt.subplot(133)
    plt.plot(s, ck, "-r", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("curvature [1/m]")

    plt.show()






