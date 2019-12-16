# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import math
import numpy as np
import matplotlib.pyplot as plt
from utils import *
from Path_GEN import *
from MPC_auv import  *
from PARAMS import *

show_animation = True

# Proportional coefficients
Kp_x = 3
Kp_y = 3
Kp_psi = 8

# Derivative coefficients
Kd_x = 15
Kd_y = 15
Kd_psi = 8


# dl 刻度尺
dl = 1 # [m]
DT1 = 1

# Trajectory
cx,cy,cpsi,ck,goal = get_switch_back_course(dl)
sp = calc_speed_profile(cx,cy,cpsi,TARGET_SPEED_U,TARGET_SPEED_V)

# T
T = len(cx)

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

def calc_ref(state,cx,cy,cpsi,pind,dl,sp):
    """
    计算轨迹上的参考点————即进行预测时，作为基准的点，只取K+1（预测时域）个
    :param state: 状态量,x,y,psi
    :param cx:
    :param cy:
    :param cpsi:
    :param ck:
    :param sp: 速度曲线
    :param dl:
    :param pind:
    :return:
    """
    xref = np.zeros((NX,T))
    dref = np.zeros((3,T)) # r

    ncourse = len(cx)

    # 计算下一个Target
    ind,_ = cal_nearest_index(state,cx,cy,cpsi,pind)

    if pind >= ind:
        ind = pind

    xref[0,0] = cx[ind]
    xref[1,0] = cy[ind]
    xref[2,0] = cpsi[ind]

    dref[0,0] = sp[0,ind]
    dref[1,0] = sp[1,ind]
    dref[2,0] = 0.0

    for i in range(T):

        xref[0,i] = cx[i]
        xref[1,i] = cy[i]
        xref[2,i] = cpsi[i]

        dref[0,i] = sp[0,i]
        dref[1,i] = sp[1,i]
        dref[2,i] = 0

    return xref,ind,dref

def pid_sim(cx,cy,cpsi,dl,initial_state,sp):
    """PID对比实验"""
    goal = [cx[-1],cy[-1]]
    state = initial_state

    # 初始化
    # 初始化 psi， 进行角度补偿
    if state.psi - cpsi[0] >= math.pi:
        state.psi -= (2.0*math.pi)
    elif state.psi - cpsi[0] <= -math.pi:
        state.psi += (2.0*math.pi)

    time = 0
    x = [state.x]
    y = [state.y]
    psi = [state.psi]
    u = [0.0]
    v = [0.0]
    r = [0.0]

    # error
    e_x = []
    e_y = []
    e_psi = []

    # e_u = []
    # e_y = []
    # e_r = []

    # target_ind,_ = cal_nearest_index(state,cx,cy,cpsi,0)

    cpsi = smooth_psi(psi)

    # while MAX_TIME >= time:
    # xref,target_ind,dref = calc_ref(state,cx,cy,cpsi,target_ind,dl,sp)

    # 初始状态
    x0 = [state.x,state.y,state.psi]

    t = 0
    # while t<=T:
    e_u_c = 0
    e_v_c = 0
    e_r_c = 0
    # for t in range(0,T):
    #     if t>0:
    #
    #         e_u = Kp_x*

    for t in range(0,T):
        print(cx[t])
        print('.')

        x_c = x[t]+(np.cos(psi[t])*u[t]-np.sin(psi[t])*v[t])*0.1
        y_c = y[t]+(np.sin(psi[t])*u[t]+np.cos(psi[t])*v[t])*0.1
        psi_c = psi[t]+(r[t])*0.1

        x.append(x_c)
        y.append(y_c)
        psi.append(psi_c)

        e_x_c = cx[t]-x_c
        e_y_c = cy[t]-y_c
        e_psi_c = cpsi[t]-psi_c
        e_x.append(e_x_c)
        e_y.append(e_y_c)
        e_psi.append(e_psi_c)

        if t >0:
            u_x = Kp_x*(e_x_c)+Kd_x*(e_x_c)
            u_y = Kp_y*(e_y_c)+Kd_y*(e_y_c)
            u_psi = Kp_psi*(e_psi_c)+Kd_psi*(e_psi_c)
        else:
            u_x = Kp_x*(e_x_c)+Kd_x*(e_x_c-e_x[t-1])
            u_y = Kp_x*(e_y_c)+Kd_y*(e_y_c-e_y[t-1])
            u_psi = Kp_psi*(e_psi_c)+Kd_psi*(e_psi_c-e_psi[t-1])

        u.append(u_x)
        v.append(u_y)
        r.append(u_psi)

        # t = t+DT1

    return x,y,psi,u,v,r

if __name__ == '__main__':
    error_x = []
    error_y = []
    initial_state = STATE(X=cx[0], Y=cy[0], psi=cpsi[0])
    x,y,psi,u,v,r = pid_sim(cx,cy,cpsi,dl,initial_state,sp)

    plt.subplot(2,2,1)
    plt.plot(cx,cy,"-r",label="spline")
    plt.plot(x, y, "-g", label='pid')
    plt.grid(True)
    plt.legend()
    # plt.show()

    plt.subplot(2,2,2)
    plt.plot(cx,"-r",label='x_d')
    plt.plot(x,"-g",label='x')
    plt.grid(True)
    plt.legend()

    plt.subplot(2,2,3)
    plt.plot(cy,"-r",label='y_d')
    plt.plot(y,"-g",label='y')

    plt.subplot(2,2,4)
    print(len(cx))
    print(len(x))
    for index,value in enumerate(cx):
        error_x.append(x[index]-value)
        error_y.append(y[index]-cy[index])
    plt.plot(error_x,"-r",label='error_x')
    plt.plot(error_y,"-g",label='error_y')

    plt.show()























