# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import pygame as pg
import numpy as np
from random import randint,uniform
import sys
import os
import random
from Settings import *
import math

np.random.seed(1) # 随机因子
SCREEN  = pg.display.set_mode((WIDTH,HEIGHT))
vec = pg.math.Vector2 # 向量申明

"""水下机器人"""
class AUV(pg.sprite.Sprite):
    def __init__(self,x,y):
        pg.sprite.Sprite.__init__(self)
        self.image = pg.Surface((TILESIZE*2,TILESIZE*5))
        self.image.fill(YELLOW)
        self.image.set_colorkey(BLUE)
        self.original_image = self.image
        self.rect = self.image.get_rect() # 获得image的包围框坐标
        # self.rect.center = (WIDTH / 2, HEIGHT / 2)  # 初始位置（AUV中心所在初始位置)
        # 初始速度x,y方向
        self.vel = vec(0,50)
        # 初始位置
        self.pos = vec(0,0)
        # 初始加速度
        self.a = vec(0,0)
        # 初始角加速度
        self.angle_a = 0
        # 初始转角
        self.rot = 0
        # 初始朝向


    def get_keys(self):
        """扫描按键"""
        self.rot_speed = 0
        self.vel = vec(0,0)
        keys = pg.key.get_pressed()
        if keys[pg.K_LEFT] or keys[pg.K_a]:
            self.rot_speed = -AUV_ROT_SPEED
        if keys[pg.K_RIGHT] or keys[pg.K_d]:
            self.rot_speed = +AUV_ROT_SPEED
        if keys[pg.K_UP] or keys[pg.K_w]:
            self.vel = vec(0,-AUV_SPEED).rotate(-self.rot) # 在当前转角方向进行up
        if keys[pg.K_DOWN] or keys[pg.K_s]:
            self.vel = vec(0,AUV_SPEED).rotate(-self.rot)

    def update(self,dt,mode,target,obstacle):
        """
        dt=clock.tick(FPS),Time Based Movement
        vx(pixel/s) * dt = vx(frame/s)
        无论FPS是多少，最终速度是自适应改变的
        ------------------
        mode:
        HAND手动操作
        AUTO自动操作
        ------------------
        """
        if mode == "HAND":
            self.get_keys()
            self.rot = (self.rot + self.rot_speed*dt)%360 # 防止超过360°
            # self.image = pg.transform.rotate(AUV,self.rot) # AUV偏航角
            self.image = pg.transform.rotate(self.original_image,self.rot)
            self.image.set_colorkey(BLUE)
            self.rect = self.image.get_rect()
            self.rect.center = self.pos
            self.pos += self.vel*dt  # 位置更新

        elif mode == "AUTO":
            x_desired,y_desired = target
            desired_vel = (target - self.pos)
            dist_to_target = desired_vel.length()
            desired_vel.normalize_ip() # 归一化

            obstacle_vel = (self.pos - obstacle)
            dist_to_ob = obstacle_vel.length()
            obstacle_vel.normalize_ip() # 归一化

            # ---- 避障策略 -----
            if dist_to_ob < OBSTACLE_CIRCLE:
                obstacle_vel = (self.pos - obstacle).normalize()*AUV_MAX_SPEED*2 # 最大速度1.5倍逃逸
                self.desired = obstacle_vel
            else:
                # ---- 抵达目标的策略 ----
                if dist_to_target < TARGET_CIRCLE:
                    desired_vel *= (dist_to_target / TARGET_CIRCLE) * AUV_MAX_SPEED
                elif dist_to_target >= TARGET_CIRCLE:
                    desired_vel *= AUV_MAX_SPEED
                self.desired = desired_vel
                # -----------------------

            steer = (self.desired - self.vel)
            if steer.length() > MAX_STEER:
                steer.scale_to_length(MAX_STEER)

            # # if self.pos != vec(0,0):
            # self.rot = rad2angle(angle_between(self.vel,self.desired))
            # # else:
            # #     self.rot = rad2angle(angle_between(POSITIVE_Y,self.vel))
            # self.image = pg.transform.rotate(self.original_image,self.rot)
            # self.rect = self.image.get_rect()

            # update ...
            self.acc = steer
            self.vel += self.acc*dt

            self.rel_x,self.rel_y = self.desired
            self.rot = 90-angle(self.rel_x,self.rel_y)
            self.image = pg.transform.rotate(self.original_image,self.rot)
            self.rect = self.image.get_rect()

            if self.vel.length() > AUV_MAX_SPEED:
                self.vel.scale_to_length(AUV_MAX_SPEED)
            self.pos += self.vel*dt
            if self.pos.x > WIDTH:
                self.pos.x = 0
            if self.pos.x < 0:
                self.pos.x = WIDTH
            if self.pos.y > HEIGHT:
                self.pos.y = 0
            if self.pos.y < 0:
                self.pos.y = HEIGHT
            self.rect.center = self.pos

    def draw_vector(self,dt):
        scale = 25 # 长度
        pg.draw.line(SCREEN,BLACK,self.pos,(self.pos+self.vel*dt*scale),3)
        pg.draw.line(SCREEN,WHITE,self.pos,(self.pos+self.desired*dt*scale),3)

"""障碍物"""
class Obstacle(pg.sprite.Sprite):
    """水下障碍物"""
    def __init__(self):
        pg.sprite.Sprite.__init__(self)
        # self.image = pg.Surface((TILESIZE*10,TILESIZE*10))
        self.image = pg.image.load(ROCKALT)
        self.image.set_colorkey(BLUE)
        # self.image.fill(OBSTACLE)
        self.rect = self.image.get_rect()
        self.rect.center = (WIDTH/2,HEIGHT/2)

    def draw_vectors(self):
        """绘制出奖励范围"""
        scale = 25
        pos = self.rect.center
        # 绘制出Obstacle的范围
        pg.draw.circle(SCREEN, RED , pos, OBSTACLE_CIRCLE, 1)

"""Target"""
class Target(pg.sprite.Sprite):
    """要寻找的目标"""
    def __init__(self):
        pg.sprite.Sprite.__init__(self)
        self.image = pg.Surface((TILESIZE*2,TILESIZE*2))
        self.image.fill(RED)
        self.rect = self.image.get_rect()
        self.rect.center = (WIDTH-5*TILESIZE,HEIGHT-5*TILESIZE)

    def draw_vectors(self):
        """绘制出奖励范围"""
        scale = 25
        pos = self.rect.center
        # 绘制出Obstacle的范围
        pg.draw.circle(SCREEN, GREEN , pos, TARGET_CIRCLE, 1)

class Reward():
    """定义奖励函数"""
    pass

"""-----------------------------------------------------------------"""
"""配置"""
def Render(mode):

    show_vectors = False
    pg.init()
    pg.mixer.init()
    screen = SCREEN
    # 设置窗口名称
    pg.display.set_caption("AUV env")
    # 设置时间
    clock = pg.time.Clock()
    # 添加sprites
    all_sprites = pg.sprite.Group()
    # ---auv---
    player = AUV(WIDTH/2,HEIGHT/2)
    print(player.rect[0:2])
    all_sprites.add(player)
    # ---obstacle---
    obstacle = Obstacle()
    all_sprites.add(obstacle)
    # ---target---
    target = Target()
    all_sprites.add(target)

    # GAME_LOOP
    running = True
    while running:
        # 设置dt
        dt = clock.tick(FPS)/1000
        pg.display.set_caption("{:.2f}".format(clock.get_fps()))
        # ---------处理事件--------------
        for event in pg.event.get():
            # 关闭window
            if event.type == pg.QUIT:
                running = False
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_v:
                    show_vectors = not show_vectors
        # ---------更新-----------------
        all_sprites.update(dt,mode,target.rect.center,obstacle.rect.center)
        # --------使能Draw/Render-------
        screen.fill(BLUE)

        """网格线绘制"""
        for x in range(0,WIDTH,TILESIZE):
            pg.draw.line(screen,LIGHTGREY,(x,0),(x,HEIGHT))
        for y in range(0,HEIGHT,TILESIZE):
            pg.draw.line(screen,LIGHTGREY,(0,y),(WIDTH,y))

        all_sprites.draw(screen)

        if show_vectors:
            # 绘制obstacle的circle
            obstacle.draw_vectors()
            target.draw_vectors()
            player.draw_vector(dt)

        pg.display.flip()
    pg.quit()

if __name__ == "__main__":
    mode = 'AUTO'
    Render(mode)