# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import pygame as pg
import random
from collections import  deque
import  os

ICON_PATH = 'D:\PycharmProjects\GYMEnv\Platformer Art Complete Pack\Base pack\Player\p1_duck.png'
vec = pg.math.Vector2

#GLOBAL
TILESIZE = 20  # 每个grid尺寸
GRIDWIDTH = 30 # 每行多少个
GRIDHEIGHT = 20 # 每列多少个
WIDTH = TILESIZE * GRIDWIDTH
HEIGHT = TILESIZE * GRIDHEIGHT

FPS = 30
WHITE = (255,255,255)
BLACK = (0,0,0)
RED = (255,0,0)
GREEN = (255,0,255)
CYAN = (0,255,255)
MAGENTA = (255,0,255)
YELLOW = (255,255,0)
DARKGRAY = (40,40,40)
LIGHTGRAY = (140,140,140)

# pygame初始化
pg.init()
screen = pg.display.set_mode((WIDTH,HEIGHT))
clock = pg.time.Clock()

class SquareGrid:
    """MAP"""
    def __init__(self,width,height):
        self.width = width
        self.height = height
        self.walls = []
        self.connections = [vec(1,0),vec(-1,0),vec(0,1),vec(0,-1)] # 移动方式

    def in_bounds(self,node):
        """界内？"""
        return 0<= node.x < self.width and 0 <= node.y < self.height

    def passable(self,node):
        return node not in self.walls

    def find_neighbors(self,node):
        neighbors = [node + connection for connection in self.connections]
        neighbors = filter(self.in_bounds,neighbors)
        neighbors = filter(self.passable,neighbors)
        return neighbors

    def draw(self):
        for wall in self.walls:
            rect = pg.Rect(wall*TILESIZE,(TILESIZE,TILESIZE))
            pg.draw.rect(screen,LIGHTGRAY,rect)

def draw_grid():
    for x in range(0,WIDTH,TILESIZE): #(0,50)
        """TILESIZE指数量"""
        pg.draw.line(screen,LIGHTGRAY,(x,0),(x,HEIGHT)) # 画x轴格线
    for y in range(0,HEIGHT,TILESIZE):
        pg.draw.line(screen,LIGHTGRAY,(0,y),(WIDTH,y)) # 画y轴格线

def draw_icons():
    start_center = (goal.x*TILESIZE + TILESIZE/2, goal.y*TILESIZE+TILESIZE/2)

def vec2int(v):
    """将小数向量分量转化为整型"""
    return (int(v.x),int(v.y))

def breadth_first_search(graph,start,end):
    """广度优先搜索算法"""
    frontier = deque()
    frontier.append(start) # 开始位置
    path = {}
    path[vec2int(start)] = None
    while len(frontier) > 0:
        #  当有frontier时
        current = frontier.popleft() # 移除当前
        if current == end:
            break
        for next in graph.find_neighbors(current):
            # 在剩余所有邻居中找到路径最短的
            if vec2int(next) not in path:
                frontier.append(next)
                path[vec2int(next)] = current - next
    return path




g = SquareGrid(GRIDWIDTH,GRIDHEIGHT)

#由鼠标点击输出坐标位置（press"m")
walls = [(5, 4), (5, 5), (5, 6), (5, 7), (5, 8), (5, 10), (5, 9), (5, 11), (5, 12), (5, 13), (5, 14), (5, 15), (5, 16), (6, 16), (6, 14), (6, 15), (6, 13), (6, 12), (6, 11), (6, 10), (6, 9), (6, 8), (7, 7), (6, 7), (6, 6), (6, 5), (6, 4), (7, 4), (7, 5), (7, 6), (7, 9), (7, 8), (7, 10), (7, 11), (7, 12), (7, 13), (7, 14), (7, 15), (7, 16), (10, 6), (10, 7), (10, 8), (10, 9), (10, 10), (10, 11), (10, 12), (10, 13), (10, 14), (10, 15), (11, 6), (11, 7), (11, 8), (11, 9), (11, 11), (11, 10), (11, 12), (11, 13), (11, 14), (11, 15), (12, 15), (12, 14), (12, 13), (12, 12), (12, 11), (12, 10), (12, 9), (12, 8), (12, 7), (12, 6), (15, 4), (15, 5), (15, 7), (15, 6), (15, 8), (15, 9), (15, 10), (15, 11), (15, 12), (15, 13), (15, 14), (15, 15), (15, 16), (16, 4), (17, 4), (18, 4), (19, 4), (20, 4), (21, 4), (22, 4), (23, 4), (24, 4), (25, 4), (26, 4), (26, 5), (25, 5), (24, 5), (23, 5), (22, 5), (21, 5), (20, 5), (19, 5), (18, 5), (16, 5), (17, 5), (16, 6), (16, 7), (16, 8), (16, 9), (16, 10), (16, 11), (16, 12), (16, 13), (16, 14), (16, 15), (16, 16), (17, 16), (18, 16), (19, 16), (20, 16), (21, 16), (22, 16), (23, 16), (24, 16), (25, 16), (26, 16), (26, 15), (25, 15), (24, 15), (23, 15), (22, 15), (21, 15), (18, 15), (17, 15), (19, 15), (20, 15), (26, 6), (26, 7), (26, 8), (26, 11), (26, 12), (26, 13), (26, 14), (25, 11), (24, 11), (23, 11), (22, 11), (21, 11), (21, 12), (22, 12), (23, 12), (24, 12), (25, 12), (25, 8), (23, 8), (22, 8), (21, 8), (20, 8), (19, 8), (18, 8), (24, 8)]
for wall in walls:
    g.walls.append(vec(wall))

# GAME_LOOP
running = True
while running:
    clock.tick(FPS)
    for event in pg.event.get():
        # 检测事件
        if event.type == pg.QUIT:
            # 关闭窗口
            running = False
        if event.type == pg.KEYDOWN:
            # 有按键按下
            if event.key == pg.K_ESCAPE:
                running = False
            if event.key == pg.K_m:
                # 将添加的wall保存下来
                print([(int(loc.x),int(loc.y)) for loc in g.walls])
        if event.type == pg.MOUSEBUTTONDOWN:
            # 按下鼠标左键
            mpos = vec(pg.mouse.get_pos())//TILESIZE # 获取按下位置(第几个?)
            if event.button == 1:
                if mpos in g.walls:
                    g.walls.remove(mpos)
                else:
                    g.walls.append(mpos)

    pg.display.set_caption("{:.2f}".format(clock.get_fps())) #显示FPS
    screen.fill(DARKGRAY)

    # render/draw
    draw_grid() #画栅格
    g.draw()
    pg.display.flip()

pg.quit()