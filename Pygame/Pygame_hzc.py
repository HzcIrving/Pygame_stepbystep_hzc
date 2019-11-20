# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import pygame
import random
import os

game_folder = os.getcwd()
player_folder = os.path.join(game_folder,'cherry.png')
player_img = pygame.image.load(player_folder)

# GLOBAL
WIDTH = 600
HEIGHT = 500
FPS = 30

# define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Player对象
class Player(pygame.sprite.Sprite):
    """继承Sprite"""
    def __init__(self):
        pygame.sprite.Sprite.__init__(self)
        self.image = player_img
        self.image.set_colorkey(BLACK)
        self.rect = self.image.get_rect() # 获得图片矩形框坐标
        self.rect.center = (WIDTH/2, HEIGHT/2) # 初始位置
        # self.vx = 120 # pixel/s
        # self.px = 0


    def update(self,dt):
        """
        dt=clock.tick(FPS),Time Based Movement
        vx(pixel/s) * dt = vx(frame/s)
        无论FPS是多少，最终速度是自适应改变的
        """
        # --------八种移动模式----------
        self.vx = 0
        self.vy = 0
        keystate = pygame.key.get_pressed()
        if keystate[pygame.K_UP]:
            self.vy = -120*dt
        if keystate[pygame.K_DOWN]:
            self.vy = 120*dt
        if keystate[pygame.K_LEFT]:
            self.vx = -120*dt
        if keystate[pygame.K_RIGHT]:
            self.vx = 120*dt
        if self.vx != 0 and self.vy != 0:
            self.vx /= 1.414
            self.vy /= 1.414
        self.rect.x += self.vx
        self.rect.y += self.vy


# 初始化pygame并创建window
pygame.init()
pygame.mixer.init()
screen = pygame.display.set_mode((WIDTH,HEIGHT))
pygame.display.set_caption("AUV env")
clock = pygame.time.Clock() # 设置时间
all_sprites = pygame.sprite.Group()
player = Player()
all_sprites.add(player)

# GAME LOOP
running = True
while running:
    # 保持循环以正确的速度运行
    dt = clock.tick(FPS)/1000 # ms

    # Process Input(events)
    for event in pygame.event.get():
        # 关闭window
        if event.type == pygame.QUIT:
            running = False

    # Update
    all_sprites.update(dt)

    # Draw/Render
    screen.fill(BLACK)

    # *after* drawing everything, flip the display
    all_sprites.draw(screen)
    pygame.display.flip()

pygame.quit()