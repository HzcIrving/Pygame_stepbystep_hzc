# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import math
import numpy as np

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
MAX_ITER = 10  # Max iteration
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
MIN_SPEED_V = -3

# --- rl params ---
GAMMA = 0.98

show_animation = True