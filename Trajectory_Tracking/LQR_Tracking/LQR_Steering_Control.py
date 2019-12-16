# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

"""
LQR(Linear Quadratic Regulator) Steering Control
- Path tracking simulation with LQR steering control and PID speed control.

Cost Function:
min J = 积分(x.T*Q*x + u.T*R*u)dt
Q: 关心状态变量
R：关心输入

概念：
1.path --- 指Vehicle/AUV... 能够沿着的线，一条特定的路径包括很多航点;
2.Speed Profile --- 速度曲线,描述了车辆的速度随着时间的变化而变化
3.Trajectory --- 轨迹，Trajectory = Path + Speed Profile
"""

import math
import sys
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la

# ====params====
lqr_Q = np.eye(5) # 初始化 等权重
lqr_R = np.eye(2) # 初始化 等权重
dt = 0.1 # time ticks/s
L = 0.5 # 车辆轴距
max_steer = np.deg2rad(45.0) # 最大转向角

show_animation = True  # 动态展示标志位

class State:
    """小车状态（可以扩充到AUV..."""
    def __init__(self,x=0.0,y=0.0,yaw=0.0,v=0.0):
        """
        :param x: 小车的x方向位置
        :param y: 小车的y方向位置
        :param yaw: 小车偏航角
        :param v: 小车后轴速度Vr
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

# 参数update
def update(state,a,delta):
    """
    :param state: 状态量 x,y,yaw
    :param a: 加速度
    :param delta: steer 转向角
    :return: 更新后的state
    """
    if delta >= max_steer:
        delta = max_steer
    if delta <= -max_steer:
        delta = -max_steer

    # 运动学方程  (Vr)
    # dot(X) = Vr * cos yaw
    # dot(Y) = Vr * sin yaw
    # dot(yaw) = Vr * tan(delta) / L >> (Vy = Vr tan delta)
    # X[k+1] = A X[k] + B u[k]
    state.x = state.x + state.v*math.cos(state.yaw)*dt
    state.y = state.y + state.v*math.sin(state.yaw)*dt
    state.yaw = state.yaw + state.v/L*math.tan(delta)*dt
    state.v = state.v + a*dt
    return state

def pi2pi(angle):
    """
    检测角度是否周期回归
    angle: 弧度
    return: 最小angle (弧度)
    """
    return (angle+math.pi)%(2*math.pi)-math.pi

def solve_dare(A,B,Q,R):
    """
    解离散的time_Algebraic Riccati方程（DARE)
    这里的x是P
    """
    x = Q
    x_next = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        x_next = A.T @ x @ A - A.T @ x @ B @ \
                 la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
        if (abs(x_next - x)).max() < eps:
            break
        x = x_next
    return x_next

def DLQR(A,B,Q,R):
    """解离散时间LQR控制器
    x[k+1] = Ax[k]+Bu[k]  --> 线性化、离散化
    cost = 积分换sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    # 首先，解黎卡提方程
    X = solve_dare(A,B,Q,R) # 求得P
    # 计算LQR增益K  u=-KX (K=k1,k2...)
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A) # 求得K

    # 特征值
    eig_result = la.eig(A-B@K) # Acl闭环矩阵特征值

    return K,X,eig_result[0]

def cal_nearest_index(state,cx,cy,cyaw):
    """
    计算目前位置后，返回下一个target的index与最近距离
    :param state:输入的状态量 x,y,yaw
    :param cx:target pos_x
    :param cy:target pos_y
    :param cyaw:target yaw
    :return:ind, e
    """
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [idx**2 + idy**2 for (idx,idy) in zip(dx,dy)] # 到Trajectory的各个点的距离的平方
    e = min(d) # 找到最小值
    ind = d.index(e) # 最小值的index索引
    e = math.sqrt(e)
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    current_yaw = math.atan2(dyl,dxl) # 当前偏航角
    e_angle = pi2pi(cyaw[ind]-math.atan2(dyl,dxl)) # 当前期望叫偏差
    if e_angle < 0:
        e *= -1 # 表示过头了！ 需要返回
    return ind, e

def lqr_speed_steering_control(state,cx,cy,cyaw,ck,pe,pth_e,sp,Q,R):
    """
    :param state: 输入的状态量（这里注意与前面不同）
        # state vector
            # x = [e, dot_e, th_e, dot_th_e, delta_v]
            # e: 到trajectory的最近距离
            # dot_e: e的倒数
            # th_e: 与trajectory的角度差
            # dot_th_e: the的倒数
            # delta_v: 目前速度与目标速度差值 v := v + at
    :param cx: 下一个target x
    :param cy: 下一个target y
    :param cyaw: 下一个target yaw
    :param ck: 下一个target时, 曲率
    ---------------------------
    :param pe: pos误差
    :param pth_e: yaw误差
    --------------------------
    :param sp: speed_profile 速度曲线
    :param Q: 状态惩罚矩阵
    :param R: 输入惩罚矩阵  >> J = ∫ x.TQx + u.TRu dt
    :return: delta,ind,e,th_e,accel
    """
    # 获得下一个目标的index和距离e
    ind,e = cal_nearest_index(state,cx,cy,cyaw)

    tv = sp[ind] # 当前期望速度
    k = ck[ind] #LQR增益

    v = state.v
    th_e = pi2pi(state.yaw - cyaw[ind]) # 误差

    # 状态向量 state_vector
    # x = [e, dot_e, th_e, dot_th_e, delta_v]
    # e: 到trajectory的最近距离
    # dot_e: e的倒数
    # th_e: 与trajectory的角度差
    # dot_th_e: the的倒数
    # delta_v: 目前速度与目标速度差值 v := v + at
    x = np.zeros((5,1))
    x[0,0] = e
    x[1,0] = (e-pe)/dt # 离散化，前向欧拉
    x[2,0] = th_e  # 偏航角误差
    x[3,0] = (th_e - pth_e)/dt # 离散化求导
    x[4,0] = v - tv # 速度差

    # 离散化状态方程X[k+1] = A*X[k] + B* U[k] 递归 --------------
    # delta很小， tan(delta)≈delta
    # A = [1.0, dt, 0.0, 0.0, 0.0
    #      0.0, 0.0, v, 0.0, 0.0]
    #      0.0, 0.0, 1.0, dt, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 1.0]
    A = np.zeros((5,5))
    A[0,0] = 1.0
    A[0,1] = dt
    A[1,2] = v
    A[2,2] = 1.0
    A[2,3] = dt
    A[4,4] = 1.0

    # B = [0.0, 0.0
    #     0.0, 0.0
    #     0.0, 0.0
    #     v/L, 0.0
    #     0.0, dt]
    B = np.zeros((5,2))
    B[3,0] = v/L
    B[4,1] = dt
    # ---------------------------------------------------

    # LQR增益K
    K,_,_ = DLQR(A,B,Q,R)

    # 输入u = [delta,a]
    # 反馈线性化 u = -kx ...
    # K = [k1,k2,k3,k4,k5]
    ustar = -K @ x

    # 计算转向输入
    ff = math.atan2(L*k,1) #  前馈转向角
    fb = pi2pi(ustar[0,0]) #  反馈转向角
    delta = ff + fb

    # 计算加速度输入
    u_a = ustar[1,0]

    return delta,ind,e,th_e,u_a

def calc_speed_profile(cyaw,target_speed):
    """速度曲线：描述了车辆的速度随时间变化的曲线"""
    speed_profile = [target_speed] * len(cyaw) # 曲线点数
    direction = 1.0 # 1.0表示正方向

    # 设置停止点
    for i in range(len(cyaw)-1):
        dyaw = abs(cyaw[i+1]-cyaw[i]) # 求偏航变化角
        switch = math.pi/4.0 <= dyaw < math.pi/2.0 # 如果dyaw的范围在45~90°之间
        if switch:
            direction *= -1 #  改变方向
        """根据yaw的变化，来确定速度的方向，可以考虑圆形轨迹"""
        if direction != 1: #若direction = -1
            speed_profile[i] = -target_speed # 速度方向是反的
        else:
            speed_profile[i] = target_speed # 速度方向正确
        if switch:
            speed_profile[i] = 0.0

    # 减速
    for i in range(40):
        speed_profile[-i] = target_speed/(50-i)
        if speed_profile[-1] <= 1.0/3.6: # minmal
            speed_profile[-1] = 1.0/3.6

    return speed_profile

def do_simulation(cx,cy,cyaw,ck,speed_profile,goal):
    """
    仿真函数
    :param cx: Trajectory _ posx
    :param cy: Trajectory _ posy
    :param cyaw: Trajectory _ yaw
    :param ck:  Trajectory _ 曲率
    :param speed_profile: 速度曲线，描述车速随时间变化而变化的曲线
    :param goal: 目标点
    :return: t,x,y,yaw,v
    """
    T = 500 # 最大仿真时间
    goal_dis = 0.3 # sqrt(dx**2 + dy**2)距离
    stop_speed = 0.05 # 停止速度

    state = State(x=-0.0,y=-0.0,yaw=0.0,v=0.0)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]

    # 初始化位置error与yaw error
    e,e_th = 0.0,0.0

    while T>= time:
        # delta: delta 转角
        # target_ind : 下一个tracking的目标target的index
        # e: 位置偏差
        # e_th : 转角偏差
        # ai: 加速度
        delta,target_ind,e,e_th,ai = lqr_speed_steering_control(
            state,cx,cy,cyaw,ck,e,e_th,speed_profile,lqr_Q,lqr_R
        )

        state = update(state,ai,delta) # 更新状态，根据lqr计算出的加速度a与转角delta

        if abs(state.v) <= stop_speed:
            target_ind += 1

        # update time
        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.sqrt((dx**2+dy**2))<=goal_dis :
            print("GOAL!...") # 达到goal点
            break

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if target_ind%1 == 0 and show_animation:
            plt.cla() # 动态作图
            plt.plot(cx,cy,"-r",label='Trajectory')
            plt.plot(x,y,"ob",label='Vehicle')
            plt.plot(cx[target_ind],cy[target_ind],"*g",label='Target')
            plt.axis("equal") # 坐标轴开
            plt.grid(True) # 网格开
            # state.v * 3.6 -> 转化为km/h
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
                      + ",target index:" + str(target_ind))
            plt.pause(0.0001)

    return t,x,y,yaw,v
























