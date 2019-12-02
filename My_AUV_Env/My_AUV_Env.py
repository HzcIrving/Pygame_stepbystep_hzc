# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

"""
ENV for target search...
"""

import pygame as pg
from Settings import *

vec = pg.math.Vector2 # 二维向量

class AUV(pg.sprite.Sprite):
    def __init__(self,game,x,y):
        self.groups = game.all_sprites #设置group
        pg.sprite.Sprite.__init__(self,self.groups)
        self.game = game
        # ---------------------------------------------
        self.image = pg.Surface((TILESIZE,TILESIZE))
        self.image.fill(BLACK) # 填充Blue
        self.rect = self.image.get_rect()
        # self.auv.center = self.pos

        # -------------------------------------------
        self.x = x*TILESIZE # pos_x
        self.y = y*TILESIZE # pos_y 坐标Xgrid尺寸
        self.vx = 0
        self.vy = 0 #speed

        self.ax = 0
        self.ay = 0
        # self.pos = vec(self.x,self.y)
        # self.vel = vec(self.vx,self.vy)
        # self.acc = vec(0,0) #初始加速度向量为0

    def get_keys(self):
        # 手动
        self.vx,self.vy = 0,0
        keys = pg.key.get_pressed()
        if keys[pg.K_LEFT] or keys[pg.K_a]:
            self.vx = -AUV_SPEED
        if keys[pg.K_RIGHT] or keys[pg.K_d]:
            self.vx = AUV_SPEED
        if keys[pg.K_UP] or keys[pg.K_w]:
            self.vy = -AUV_SPEED
        if keys[pg.K_DOWN] or keys[pg.K_s]:
            self.vy = AUV_SPEED
        if self.vx!=0 and self.vy!=0:
            """SMOOTH"""
            self.vx *= 0.7071
            self.vy *= 0.7071

    def update(self):
        """UPDATE"""
        # self.rect.x = self.x * TILESIZE
        # self.rect.y = self.y * TILESIZE
        self.get_keys()
        self.x += self.vx * self.game.dt  # x更新
        self.y += self.vy * self.game.dt
        self.rect.x = self.x
        self.rect.y = self.y

    def render(self):
        pass

class Obstacle(pg.sprite.Sprite):
    """障碍物"""
    def __init__(self,game,x,y):
        self.groups = game.all_sprites,game.walls
        pg.sprite.Sprite.__init__(self,self.groups)
        self.game = game
        self.image = pg.Surface((TILESIZE,TILESIZE))
        self.image.fill(OBSTACLE)
        self.rect = self.image.get_rect()
        self.x = x
        self.y = y
        self.rect.x = x * TILESIZE
        self.rect.y = y * TILESIZE



