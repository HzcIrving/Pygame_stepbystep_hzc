# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import os
import sys
import pygame as pg

# Define RGB
WHITE = (255,255,255)
WHITE_LINE = (176,226,255)
BLACK = (0,0,0)
DARKGRAY = (40,40,40)
LIGHTGREY = (100,100,100)
GREEN = (0,255,0)
RED = (255,0,0)
YELLOW = (255,255,0)
BLUE = (0,191,255)
OBSTACLE = (130,90,43)

# 窗口设置
WIDTH = 800
HEIGHT = 600
FPS = 30
TITLE = "AUV Target Searching"
BGCOLOR = DARKGRAY

TILESIZE = 10
GRIDWIDTH = WIDTH/TILESIZE # 宽80
GRIDHEIGHT = HEIGHT/TILESIZE # 长60

AUV_SPEED = 50 # pixel?
AUV_MAX_SPEED = 80
AUV_ROT_SPEED = 10 # ??

# AUV ----

# Obstacle ---
ENV_PATH = os.getcwd()
OBSTACLE_PATH = os.path.join(ENV_PATH,'Pic')
ROCK = os.path.join(OBSTACLE_PATH,'rock.png')
ROCKALT = os.path.join(OBSTACLE_PATH,'rockAlt.png')
ROCK2 = os.path.join(ENV_PATH,'dirtCaveRockLarge.png')
ROCK3 = os.path.join(ENV_PATH,'stoneCaveRockLarge.png')
print(OBSTACLE_PATH)

# 各个范围
# 障碍物半径
OBSTACLE_CIRCLE = TILESIZE*10
TARGET_CIRCLE = TILESIZE*5

# 模式
MODE = "HAND"

# 最大转向力
MAX_STEER = 40
POSITIVE_Y = (0,1)


"""
常用工具
"""
import numpy as np
import math

def angle_between(v1,v2):
    """返回弧度制"""
    v1_u = v1/np.linalg.norm(v1) # 单位向量
    v2_u = v2/np.linalg.norm(v2)
    return np.arccos(np.clip(np.dot(v1_u,v2_u),-1.0,1.0))

def angle(x,y):
    """已知AUV距target之间相对距离，求中间夹角"""
    angle = math.atan2(y,x)
    angle = rad2angle(angle)
    return angle

def rad2angle(rad):
    """弧度转角度"""
    return (rad/(2*np.pi))*360

if __name__ == "__main__":
    a = angle_between(np.array([0,1]),np.array([-1,1]))
    b = rad2angle(a)
    c = angle(4,4)
    print(c)
    print(b)

    print([0.0]*5)