# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

"""
This Demo help us to use our sprite to seek the target position.
"""

import pygame as pg
from random import randint
from random import uniform

vec = pg.math.Vector2 # 二维向量

# GLOBAL
WIDTH = 800
HEIGHT = 600
FPS = 60
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
DARKGRAY = (40, 40, 40)

# AUV Properties
AUV_WIDTH = 32
AUV_HEIGHT = 22
MAX_SPEED = 5

MAX_FORCE = 0.1
APPROACH_RADIUS = 120  # Target感知范围,在Target范围附近,STEER Force会减小，防止过冲

class AUV(pg.sprite.Sprite):
    """AUV模拟追踪鼠标"""
    def __init__(self):
        self.groups = all_sprites
        pg.sprite.Sprite.__init__(self,self.groups)
        self.image = pg.Surface((AUV_WIDTH,AUV_HEIGHT))
        self.rect = self.image.get_rect()
        self.image.fill(YELLOW) # 填充颜色

        #----------------------------------
        # 向量参
        # 初始位置随机
        self.pos = vec(randint(0,WIDTH),randint(0,HEIGHT))
        # 初始只有x方向速度，且伴有0~360的随机抖动
        self.vel = vec(MAX_SPEED,0).rotate(uniform(0,360))
        # 初始加速度为0
        self.acc = vec(0,0)
        #----------------------------------
        self.rect.center = self.pos

    def follow_mouth(self):
        """获得鼠标位置进行追踪"""
        mpos = pg.mouse.get_pos()
        self.acc = (mpos-self.pos).normalize()*0.5

    def seek(self,target):
        """steer带着AUV前往target"""
        self.desired = (target - self.pos).normalize()*MAX_SPEED
        steer = (self.desired - self.vel) # vector减
        if steer.bit_length() > MAX_FORCE:
            steer.scale_to_length(MAX_FORCE)
        return steer

    def seek_with_approach(self,target):
        """steer带着AUV前往target,在进入target范围内后，降低SPEED，精准到达，减小震荡"""
        self.desired = (target - self.pos)
        dist = self.desired.length()
        self.desired.normalize_ip() # 直接对desired vector normalize
        if dist < APPROACH_RADIUS:
            self.desired *= dist / APPROACH_RADIUS * MAX_SPEED # 降低速度
        else:
            self.desired *= MAX_SPEED
        steer = (self.desired-self.vel)
        if steer.length() > MAX_FORCE:
            steer.scale_to_length(MAX_FORCE)
        return steer

    def update(self):
        # 追踪鼠标
        self.acc = self.seek_with_approach(pg.mouse.get_pos())
        # 运动学方程
        self.vel += self.acc
        if self.vel.length() > MAX_SPEED:
            self.vel.scale_to_length(MAX_SPEED)
        self.pos += self.vel

        # SCREEN LOOP
        if self.pos.x > WIDTH:
            self.pos.x = 0
        if self.pos.x < 0:
            self.pos.x = WIDTH
        if self.pos.y > HEIGHT:
            self.pos.y = 0
        if self.pos.y < 0:
            self.pos.y = HEIGHT

        # 更新AUV位置
        self.rect.center = self.pos

    def draw_vectors(self):
        """按下V，显示向量"""
        scale = 25 # 25个pixel?
        # 速度向量
        pg.draw.line(screen,GREEN,self.pos,(self.pos+self.vel*5),5)
        # 期望Target向量
        pg.draw.line(screen,RED,self.pos,(self.pos+self.desired*scale),5)
        # Target范围半径
        pg.draw.circle(screen,BLUE,pg.mouse.get_pos(),APPROACH_RADIUS,2)


# SETUP
# 1.初始化
pg.init()
screen = pg.display.set_mode((WIDTH,HEIGHT))
clock = pg.time.Clock()

# sprites
all_sprites = pg.sprite.Group()
AUV()
paused = False
show_vectors = False
running = True

# GAME LOOP ...
while running:
    clock.tick(FPS)

    # Process/event
    for event in pg.event.get():
        if event.type == pg.QUIT:
            """关闭游戏"""
            running = False
        if event.type == pg.KEYDOWN:
            if event.key == pg.K_ESCAPE:
                running = False
            if event.key == pg.K_SPACE:
                """暂停游戏"""
                paused = not paused  # 采用not就是可以repeat
            if event.key == pg.K_v:
                show_vectors = not show_vectors
            if event.key == pg.K_m:
                AUV()

    # update
    if not paused:
        all_sprites.update()
    # 显示FPS
    pg.display.set_caption("{:.2f}".format(clock.get_fps()))
    screen.fill(DARKGRAY)

    # draw/render
    all_sprites.draw(screen)
    if show_vectors:
        for sprite in all_sprites:
            sprite.draw_vectors()
    pg.display.flip()

pg.quit()







