# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

"""
用于RL连续控制的强化学习环境
AUV的Trajectory Tracking...
输入(action): u = A = [u,v,r].T
状态(state): X = S = [X,Y,psi].T
"""
import math
import numpy as np
from CubicSpline import *
from PARAMS import *
from utils import *

import gym
from gym import spaces
from gym.utils import seeding

traj_x = []
traj_y = []


def get_switch_back_course(dl):
    """
    生成路径
    :param dl:
    :return:
    """
    ax = [0.0,30.0,6.0,20.0,35.0]
    ay = [0.0,0.0,20.0,35.0,20.0]
    goal = [ax[-1],ay[-1]]
    cx,cy,cpsi,ck,s = calc_spline_course(ax,ay,ds=dl)

    cpsi = smooth_psi(cpsi) # 平滑操作

    return cx,cy,cpsi

class AUV_TRACKING(object):


    def __init__(self,cx,cy,cpsi):
        """
        :param cx: path的x
        :param cy:  path的y
        :param cpsi: path的yaw
        """
        # u
        self.min_action_u = MIN_SPEED_U
        self.max_action_u= MAX_SPEED_U
        # v
        self.min_action_v = MIN_SPEED_V
        self.max_action_v = MAX_SPEED_V
        # r
        self.min_action_r = MIN_STEER_SPEED
        self.max_action_r = MAX_STEER_SPEED

        self.min_action = np.array([self.min_action_u,self.min_action_v,self.min_action_r])
        self.max_action = np.array([self.max_action_u,self.max_action_v,self.max_action_r])

        self.action_space = spaces.Box(low=self.min_action,high=self.max_action,dtype=np.float32)

        # observation
        self.state = np.zeros((3,0))
        self.desired_x = cx
        self.desired_y = cy
        self.yaw = cpsi

        # params
        self.gamma = GAMMA

        # goal
        self.goal = [self.desired_x[-1],self.desired_y[-1]]


    def seed(self,seed=None):
        """随机因子"""
        self.np_random,seed = seeding.np_random(seed)
        return [seed]

    def step(self,action):
        """
        :param action:[u_t,v_t,r_t]
        :return:
        """
        # 状态量
        postion_X = self.state[0]
        postion_Y = self.state[1]
        Yaw = self.state[2]

        traj_x.append(postion_X)
        traj_y.append(postion_Y)

        if (action[0]>=self.max_action_u): action[0] = self.max_action_u
        if (action[0]<=self.min_action_u): action[0] = self.min_action_u
        if (action[1]>=self.max_action_v): action[1] = self.max_action_v
        if (action[1]<=self.min_action_v): action[1] = self.min_action_v
        if (action[2]>=self.max_action_r): action[2] = self.max_action_r
        if (action[2]<=self.min_action_r): action[2] = self.min_action_r

        postion_X +=  DT*(np.cos(Yaw)*action[0]-np.sin(Yaw)*action[1])
        postion_Y +=  DT*(np.sin(Yaw)*action[0]+np.cos(Yaw)*action[1])
        Yaw += DT*(action[2])

        # check goal
        dx = postion_X - self.goal[0]
        dy = postion_Y - self.goal[1]
        d = math.sqrt(dx**2+dy**2)

        if d < 0.01:
            done = True

        delta_r = (desired_x-postion_X) + (desired_y-postion_Y) + (desired_yaw-Yaw)
        rewards = (1/np.sqrt(2*np.pi))*np.exp(-1*(delta_r**2/2))

        self.state[0] = postion_X
        self.state[1] = postion_Y
        self.state[2] = Yaw

        return self.state, rewards, done, {}

    def reset(self):
        self.state = np.zeros((3,)) # 初始化
        return self.state








