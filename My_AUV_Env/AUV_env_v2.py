# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import numpy as np
import pygame as pg
from Settings import *
import time
import matplotlib.pyplot as plt
import os
import sys

# GLOBAL
SCREEN = pg.display.set_mode((WIDTH,HEIGHT))
GOAL = (WIDTH-5*TILESIZE,HEIGHT-5*TILESIZE)
ACTION_BOUND = [-1,1] # 将action范围normalize到-1~+1
STATE_DIM = 5 #x,y,yaw,v,delta
ACTION_DIM = 2 # 速度v,转角delta
vec = pg.math.Vector2

class Obstacle(pg.sprite.Sprite):
    """障碍物"""
    def __init__(self):
        pg.sprite.Sprite.__init__(self)
        self.image = pg.image.load(ROCKALT)
        self.image.set_colorkey(BLUE)
        self.rect = self.image.get_rect()
        self.rect.center = (WIDTH/2,HEIGHT/2) # 坐标位置
    def draw_vectors(self):
        """输出障碍物范围"""
        pos = self.rect.center
        pg.draw.circle(SCREEN,RED,pos,OBSTACLE_CIRCLE,2)

class Goal(pg.sprite.Sprite):
    """障碍物"""
    def __init__(self):
        pg.sprite.Sprite.__init__(self)
        self.image = pg.Surface((TILESIZE*2,TILESIZE*2))
        self.image.fill(RED)
        self.rect = self.image.get_rect()
        self.rect.center = GOAL

    def draw_vectors(self):
        """输出障碍物范围"""
        pos = self.rect.center
        pg.draw.circle(SCREEN,GREEN,pos,TARGET_CIRCLE,2)

class AUV(pg.sprite.Sprite):
    """AUV本体"""
    def __init__(self):
        pg.sprite.Sprite.__init__(self)
        self.image = pg.Surface((TILESIZE*2,TILESIZE*3))
        self.image.fill(YELLOW)
        self.rect = self.image.get_rect()
        self.rect.center = ((WIDTH/2,HEIGHT/2))
        self.pos = vec(0,0)
        self.vec = vec(0,0)
        self.acc = vec(0,0)
        self.rot = 0


class AUV_ENV(pg.sprite.Sprite):
    """
    AUV_目标搜索强化学习环境
    """
    def __init__(self):
        pg.sprite.Sprite.__init__(self)
        self.action_bound = ACTION_BOUND
        self._build_env()
        self.dt = self.clock.tick(FPS) #1/30 s/frame 刷新率
        self.AUV = AUV()

    def _build_env(self):
        """搭建环境"""
        show_vectors = False
        pg.init()
        pg.mixer.init()
        self.screen = SCREEN
        # 窗口名称
        pg.display.set_caption("AUV_env")
        # 设置时间
        self.clock = pg.time.Clock()
        # 添加sprites
        self.all_sprites = pg.sprite.Group()
        # ----obstacle----
        self.obstacle = Obstacle()
        self.all_sprites.add(self.obstacle)
        # ----Goal----
        self.goal = Goal()
        self.all_sprites.add(self.goal)
        # ----AUV----
        self.all_sprites.add(self.AUV)
        self.running = True
        while self.running:
            self.events()
            self.update()
            self.draw()
        pg.quit()


    def events(self):
        """窗口事件处理"""
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.running = False
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_r: # 手动复位
                    self.reset()

    def update(self):
        """更新窗口动态"""
        self.all_sprites.update()

    def draw(self):
        """窗口sprite可视化"""
        self.screen.fill(BLUE)
        # 网格线绘制
        for x in range(0,WIDTH,TILESIZE):
            pg.draw.line(self.screen,LIGHTGREY,(x,0),(x,HEIGHT))
        for y in range(0,HEIGHT,TILESIZE):
            pg.draw.line(self.screen,LIGHTGREY,(0,y),(WIDTH,y))

        self.all_sprites.draw(self.screen)
        pg.display.flip()

    def reset(self):
        """重置环境"""
        self.AUV.vel = vec(0,0)
        self.AUV.pos = vec(0,0)
        self.AUV.acc = vec(0,0)
        self.AUV.rot = 0
        self.AUV.rect.center = (self.AUV.pos)

    def step(self,action):
        """执行action，返回s_,reward,done"""
        done = False
        current_pos = self.AUV.rect.center # 获取水下机器人当前位置
        current_pos

    def render(self):
        """使能"""
        pass

if __name__ == '__main__':
    AUV_ENV()
