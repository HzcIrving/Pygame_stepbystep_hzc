# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

"""
通过MPC算法实现AUV的轨迹跟踪

输入v=[u,v,r];
状态X=[x,y,ψ];

控制u,v,r,使auv跟踪上已经给定的期望轨迹yd=[xd,yd,ψd]；最终
使得实际轨迹和期望轨迹之间的误差渐进收敛到0;

"""
import cvxpy
import matplotlib.pyplot as plt
import math
import numpy as np
import sys
import CubicSpline
from utils import *

NX = 3 # 状态量 [x,y,ψ]
NU = 3 # 输入量 [u,v,r]
K = 5 # horizon length

# mpc参数
R = np.diag([0.01,0.01,0.01]) # input cost matrix
Rd = np.diag([0.01,0.01,0.1]) # input difference cost matrix
Q = np.diag([1.0,1.0,0.5]) # state cost matrix
Qf = Q # state final matrix

# GLOBAL
# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.1  # iteration finish param

TARGET_SPEED = 10.0 / 3.6 - 0.5   # [m/s] target speed
TARGET_SPEED_U = 2
TARGET_SPEED_V = 1.5
N_IND_SEARCH = 10  # Search index number
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time


DT = 0.2  # [s] time tick

# AUV parameters
LENGTH = 4.5/2  # [m]
WIDTH = 2.0/2  # [m]
# BACKTOWHEEL = 1.0  # [m]
# WHEEL_LEN = 0.3  # [m]
# WHEEL_WIDTH = 0.2  # [m]
# TREAD = 0.7  # [m]
# WB = 2.5  # [m]

MAX_STEER_SPEED = 2 # maximum steering angle [rad/s]
MIN_STEER_SPEED= -2
MAX_DSTEER_SPE = 0.25
MIN_DSTEER_SPE = -0.25

MAX_SPEED_U = 3 # maximum speed [m/s]
MIN_SPEED_U = -3# minimum speed [m/s]
MAX_SPEED_V = 2
MIN_SPEED_V = -2

show_animation = True

class STATE:
    """
    AUV STATE CLASS
    """
    def __init__(self,X=0,Y=0,psi=0):
        self.x = X
        self.y = Y
        self.psi = psi

def get_linear_model_matrix(psi,u,v):
    """u,v,psi为轨迹上的期望值"""
    A = np.zeros((NX,NX))
    A[0,0] = 1
    A[1,1] = 1
    A[2,2] = 1
    A[0,2] = DT*(-u*np.sin(psi)-v*np.cos(psi))
    A[1,2] = DT*(u*np.cos(psi)-v*np.sin(psi))

    B = np.zeros((NX,NU))
    B[0,0] = DT*np.cos(psi)
    B[0,1] = -1*DT*np.sin(psi)
    B[1,0] = DT*np.sin(psi)
    B[1,1] = DT*np.cos(psi)
    B[2,2] = DT

    return A,B

def update_state(state,u,v,r):
    """状态更新"""
    if u>MAX_SPEED_U:
        u = MAX_SPEED_U
    elif u<MIN_SPEED_U:
        u = MIN_SPEED_U

    if v>MAX_SPEED_V:
        v = MAX_SPEED_V
    elif v<MIN_SPEED_V:
        v = MIN_SPEED_V

    if r>MAX_STEER_SPEED:
        r = MAX_STEER_SPEED
    elif r<MIN_STEER_SPEED:
        r = MIN_STEER_SPEED

    state.x = state.x + DT*(np.cos(state.psi)*u-np.sin(state.psi)*v)
    state.y = state.y + DT*(np.sin(state.psi)*u+np.cos(state.psi)*v)
    state.psi = state.psi + DT*(r)

    return state

def predict_motion(x0,ou,ov,or_,xref):
    """
    运动预测
    :return xbar 预测状态值（从1到K+1)
    """
    xbar = xref*0 # 初始化
    for i,_ in enumerate(x0):
        xbar[i,0] = x0[i] #初始化
    state = STATE(X=x0[0],Y=x0[1],psi=x0[2])
    for (ui,vi,ri,i) in zip(ou,ov,or_,range(1,K+1)):
        state = update_state(state,ui,vi,ri) # 更新状态
        xbar[0,i] = state.x
        xbar[1,i] = state.y
        xbar[2,i] = state.psi
    return xbar

def linear_mpc_control(xref,xbar,x0,dref):
    """
    优化问题，滚动优化解X与V
    :param xref: 参考状态
    :param xbar: 操作点(预测状态值)
    :param x0: 初始状态
    :param vref:
    """
    X = cvxpy.Variable((NX,K+1)) # 状态量 到 k+1
    V = cvxpy.Variable((NU,K)) # 输入量  到k

    cost = 0.0
    constraints = [] # 约束

    for k in range(K):
        cost += cvxpy.quad_form(V[:,k],R)

        if k != 0:
            cost += cvxpy.quad_form(xref[:,k]-X[:,k],Q)

        # 输入psi,u,v
        # A,B = get_linear_model_matrix(xbar[0,k],xbar[1,k],xbar[2,k]) (x)
        A,B = get_linear_model_matrix(xbar[2,k],xbar[0,k],dref[1,k]) # (√)
        constraints += [X[:,k+1]-xref[:,k+1] == A*(X[:,k]-xref[:,k])+B*(V[:,k]-dref[:,k])]

        if k < (K-1):
            # △v
            cost+=cvxpy.quad_form(V[:,k+1]-V[:,k],Rd)
            constraints += [cvxpy.abs(V[2,k+1]-V[2,k])<=MAX_DSTEER_SPE]

    # final cost
    cost += cvxpy.quad_form(xref[:,K]-X[:,K],Qf)

    constraints+=[cvxpy.abs(V[0,:])<=MAX_SPEED_U]
    # constraints+=[V[0,:]>=MIN_SPEED_U]
    constraints+=[cvxpy.abs(V[1,:])<=MAX_SPEED_V]
    # constraints+=[V[1,:]>=MIN_SPEED_V]
    constraints+=[cvxpy.abs(V[2,:])<=MAX_STEER_SPEED]
    # constraints+=[V[2,:]<=MIN_STEER_SPEED]
    constraints+=[X[:,0]==x0]  #初始状态
    # constraints+=[X[:,0]]

    prob = cvxpy.Problem(cvxpy.Minimize(cost),constraints) # 最小化object cost
    prob.solve(solver = cvxpy.ECOS,verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_narray_from_matrix(X.value[0, :])
        oy = get_narray_from_matrix(X.value[1, :])
        opsi = get_narray_from_matrix(X.value[2, :])
        ou = get_narray_from_matrix(V.value[0, :])
        ov = get_narray_from_matrix(V.value[1, :])
        or_ = get_narray_from_matrix(V.value[2, :])

    else:
        print("Error: Cannot solve mpc..")
        ox,oy,opsi,ou,ov,or_ = None, None, None, None, None, None

    return ox,oy,opsi,ou,ov,or_

def iterative_linear_MPC(xref,x0,dref,ou,ov,or_):
    """
    滚动迭代:MPC contorl with updating operational point iteraitvely
    MPC控制与迭代更新操作点
    """
    if ou is None or ov is None or or_ is None:
        ou = [0.0]*K # 0,0,0,0,0
        ov = [0.0]*K
        or_ = [0.0]*K

    for i in range(MAX_ITER):
        xbar = predict_motion(x0,ou,ov,or_,xref)
        pou,pov,por = ou[:],ov[:],or_[:]

        # 通过解目标函数，得到下一个控制量以及下一时刻的状态
        ox,oy,opsi,ou,ov,or_ = linear_mpc_control(xref,xbar,x0,dref)

        du = sum(abs(ou-pou))+sum(abs(ov-pov))+sum(abs(or_-por))

        if du<=DU_TH:
            break  #  输入变化量小于阈值，就可退出

    else:
        print("迭代次数用尽...")

    return ox,oy,opsi,ou,ov,or_

def do_simulation(cx,cy,cpsi,ck,dl,initial_state,sp):
    """
    仿真
    :param cx: 轨迹path x坐标
    :param cy: 轨迹path y坐标
    :param cpsi: 轨迹path 的psi角
    :param ck:
    :param sp: 速度曲线
    :param dl: course 刻度(m)
    :param initial_state: 初始状态
    :return:
    """
    goal = [cx[-1],cy[-1]] # 最后一个点为goal
    state = initial_state

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
    t = [0.0]

    target_ind,_ = cal_nearest_index(state,cx,cy,cpsi,0)

    ou = None
    ov = None
    or_ = None

    cpsi = smooth_psi(cpsi)

    while MAX_TIME >= time:
        xref , target_ind, dref = calc_ref_trajectory(state,cx,cy,cpsi,target_ind,dl,sp)

        x0 = [state.x,state.y,state.psi]  # 当前状态

        ox,oy,opsi, ou, ov, or_= iterative_linear_MPC(xref,x0,dref,ou,ov,or_)

        if or_ is not None:
            ui = ou[0]
            vi = ov[0]
            ri = or_[0]

        state = update_state(state,ui,vi,ri) # 状态更新,得到新的state
        time = time + DT

        x.append(state.x)
        y.append(state.y)
        psi.append(state.psi)
        t.append(time)
        u.append(ui)
        v.append(vi)
        r.append(ri)

        if check_goal(state,goal,target_ind,len(cx)):
            print("GOAL")
            break  # 达到最终目标点

        if show_animation:
            plt.cla()
            if ox is not None:
                plt.plot(ox,oy,"xr",label="MPC")
            plt.plot(cx,cy,"-r",label="course")
            plt.plot(x,y,"ob",label="AUV_trajectory")
            plt.plot(xref[0,:],xref[1,:],"xk",label="xref")
            plt.plot(cx[target_ind],cy[target_ind],"xg",label="target")
            plt.legend()
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:"+str(round(time,2))+",speed[km/h]:"+str(round(math.sqrt(ou[-1]**2+ov[-1]**2)*3.6,2)))
            plt.pause(0.00001)

    return t,x,y,psi,u,v,r




















