# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

from MPC_auv import *
import CubicSpline

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
N_IND_SEARCH = 10  # Search index number
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time


DT = 0.1  # [s] time tick

# AUV parameters
LENGTH = 4.5/2  # [m]
WIDTH = 2.0/2  # [m]
# BACKTOWHEEL = 1.0  # [m]
# WHEEL_LEN = 0.3  # [m]
# WHEEL_WIDTH = 0.2  # [m]
# TREAD = 0.7  # [m]
# WB = 2.5  # [m]

MAX_STEER_SPEED = 2 # maximum steering angle [rad/s]
MIN_STEER_SPEED= 0
MAX_DSTEER_SPE = 0.25
MIN_DSTEER_SPE = 0

MAX_SPEED_U = 3 # maximum speed [m/s]
MIN_SPEED_U = STOP_SPEED# minimum speed [m/s]
MAX_SPEED_V = 2
MIN_SPEED_V = STOP_SPEED

# show_animation = True

def pi2pi(angle):
    """转角修正"""
    while(angle>math.pi):
        # 转角>180°，改成x-360°转角，即反方向转
        angle = angle - 2.0*np.pi
    while(angle<-math.pi):
        angle = angle + 2.0*np.pi # 同理
    return angle

def smooth_psi(psi):
    """平滑psi"""
    for i in range(len(psi)-1):
        dpsi = psi[i+1] - psi[i]
        while dpsi >= math.pi/2.0 : #  大于90°
            psi[i+1] -= math.pi/2.0
            dpsi = psi[i+1]-psi[i]
        while dpsi <= -math.pi/2.0:
            psi[i+1] += math.pi/2.0
            dpsi = psi[i+1] - psi[i]
    return psi

def get_narray_from_matrix(x):
    """从矩阵中得到narray"""
    return np.array(x).flatten()

def cal_nearest_index(state,cx,cy,cpsi,pind):
    """
    计算目前位置后，返回下一个target的index与最近距离
    :param state: 状态量
    :param cx: trajectory的x
    :param cy: trajectory的y
    :param cpsi: trajectory的psi
    :param pind: 上一个目标点的index
    :return:
    """
    dx = [state.x-icx for icx in cx[pind:(pind+N_IND_SEARCH)]]
    dy = [state.y-icy for icy in cy[pind:(pind+N_IND_SEARCH)]]
    d = [idx**2+idy**2 for(idx,idy) in zip(dx,dy)]
    e = min(d) # 最小距离即是下个target
    ind = d.index(e)+pind
    e = math.sqrt(e)

    dxl = cx[ind] - state.x # x方向误差
    dyl = cy[ind] - state.y # y方向误差

    dangle = pi2pi(cpsi[ind]-math.atan2(dyl,dxl))
    if dangle < 0:
        e*=-1
    return ind,e

def calc_speed_profile(cx,cy,cpsi,target_speed_u,target_speed_v):

    target_speed = np.zeros((2,len(cx)))
    # speed_profile = [target_speed] * len(cx) # 初始化
    target_speed[0,:] = target_speed_u
    target_speed[1,:] = target_speed_v

    speed_profile = target_speed

    direction = 1.0 # forward

    # 设置stop_point
    for i in range(len(cx)-1):
        dx = cx[i+1] - cx[i]
        dy = cy[i+1] - cy[i]

        move_direction = math.atan2(dy,dx) # 移动方向

        if dx!= 0.0 and dy!= 0.0:
            dangle = abs(pi2pi(move_direction-cpsi[i])) # 角度变化
            if dangle>=math.pi/4.0 : # 大于45°
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[0,i] = -target_speed_u
            speed_profile[1,i] = -target_speed_v
        else:
            speed_profile[0,i] = target_speed_u
            speed_profile[1,i] = target_speed_v

    # final state
    speed_profile[0,-1] = 0.0
    speed_profile[1,-1] = 0.0

    return  speed_profile

def calc_ref_trajectory(state,cx,cy,cpsi,pind,dl,sp):
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
    xref = np.zeros((NX,K+1))
    dref = np.zeros((3,K+1)) # r

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


    travel = 0.0

    for i in range(K+1):
        travel += math.sqrt(dref[0,0]**2+dref[1,0]**2)*DT
        dind = int(round(travel/dl))

        if (ind+dind)<ncourse:
            xref[0,i] = cx[ind+dind]
            xref[1,i] = cy[ind+dind]
            xref[2,i] = cpsi[ind+dind]
            dref[0,i] = sp[0,ind+dind] # 参考的r都为0，船始终跟着trajectory
            dref[1,i] = sp[1,ind+dind]
            dref[2,i] = 0.0

        else:
            """Final state"""
            xref[0,i] = cx[ncourse-1]
            xref[1,i] = cy[ncourse-1]
            xref[2,i] = cpsi[ncourse-1]
            dref[0,i] = sp[0,ncourse-1]
            dref[1,i] = sp[1,ncourse-1]
            dref[2,i] = 0.0

    return xref,ind,dref

def check_goal(state,goal,tind,nind):
    """
    检查目标点
    tind: target点的index
    nind: index的数量(len(cx))
    """
    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.sqrt(dx**2+dy**2)

    isgoal = (d<=GOAL_DIS)

    if abs(tind-nind) >= 5:
        """如果target的索引-总索引个数>5个"""
        isgoal = False # 还没到！

    isstop = (state.x-goal[0]<=0.1 and state.y-goal[1]<=0.1)

    if isgoal and isstop:
        return True

    return False

# def auv(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover
#
#     outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
#                         [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
#
#     fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
#                          [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])
#
#     rr_wheel = np.copy(fr_wheel)
#
#     fl_wheel = np.copy(fr_wheel)
#     fl_wheel[1, :] *= -1
#     rl_wheel = np.copy(rr_wheel)
#     rl_wheel[1, :] *= -1
#
#     Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
#                      [-math.sin(yaw), math.cos(yaw)]])
#     Rot2 = np.array([[math.cos(steer), math.sin(steer)],
#                      [-math.sin(steer), math.cos(steer)]])
#
#     fr_wheel = (fr_wheel.T.dot(Rot2)).T
#     fl_wheel = (fl_wheel.T.dot(Rot2)).T
#     fr_wheel[0, :] += WB
#     fl_wheel[0, :] += WB
#
#     fr_wheel = (fr_wheel.T.dot(Rot1)).T
#     fl_wheel = (fl_wheel.T.dot(Rot1)).T
#
#     outline = (outline.T.dot(Rot1)).T
#     rr_wheel = (rr_wheel.T.dot(Rot1)).T
#     rl_wheel = (rl_wheel.T.dot(Rot1)).T
#
#     outline[0, :] += x
#     outline[1, :] += y
#     fr_wheel[0, :] += x
#     fr_wheel[1, :] += y
#     rr_wheel[0, :] += x
#     rr_wheel[1, :] += y
#     fl_wheel[0, :] += x
#     fl_wheel[1, :] += y
#     rl_wheel[0, :] += x
#     rl_wheel[1, :] += y
#
#     plt.plot(np.array(outline[0, :]).flatten(),
#              np.array(outline[1, :]).flatten(), truckcolor)
#     plt.plot(np.array(fr_wheel[0, :]).flatten(),
#              np.array(fr_wheel[1, :]).flatten(), truckcolor)
#     plt.plot(np.array(rr_wheel[0, :]).flatten(),
#              np.array(rr_wheel[1, :]).flatten(), truckcolor)
#     plt.plot(np.array(fl_wheel[0, :]).flatten(),
#              np.array(fl_wheel[1, :]).flatten(), truckcolor)
#     plt.plot(np.array(rl_wheel[0, :]).flatten(),
#              np.array(rl_wheel[1, :]).flatten(), truckcolor)
#     plt.plot(x, y, "*")




