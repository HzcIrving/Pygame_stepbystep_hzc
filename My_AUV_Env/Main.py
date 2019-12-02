# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import pygame as pg
import sys
from Settings import *
from os import path
from My_AUV_Env import *

class Game:
    def __init__(self):
        """初始化"""
        pg.init()
        self.screen = pg.display.set_mode((WIDTH,HEIGHT))
        pg.display.set_caption(TITLE)
        self.clock = pg.time.Clock()
        pg.key.set_repeat(500,100) # ???
        self.load_data()

    def load_data(self):
        pass

    def new(self):
        """初始化所有变量，配置好环境"""
        self.all_sprites = pg.sprite.Group()
        self.walls = pg.sprite.Group()
        self.player = AUV(self,100,100)

        self.all_sprites.add(self.player)

        for x in range(10,100):
            Obstacle(self,x,5)

    def run(self):
        # loop
        self.playing = True
        while self.playing:
            self.dt = self.clock.tick(FPS)/1000 # 帧/ms
            self.events()
            self.update()
            self.draw()

    def quit(self):
        """退出"""
        pg.quit()
        sys.exit()

    def update(self):
        """更新"""
        self.all_sprites.update()

    def draw_grid(self):
        for x in range(0,WIDTH,TILESIZE):
            pg.draw.line(self.screen,LIGHTGREY,(x,0),(x,HEIGHT))
        for y in range(0,HEIGHT,TILESIZE):
            pg.draw.line(self.screen,LIGHTGREY,(0,y),(WIDTH,y))

    def draw(self):
        # 显示FPS
        pg.display.set_caption("{:.2f}".format(self.clock.get_fps()))

        self.screen.fill(BLUE)
        # self.draw_grid()
        self.all_sprites.draw(self.screen)
        pg.display.flip()

    def events(self):
        # 将所有的事件都打包放在此处
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.quit()
            if event.type == pg.KEYDOWN: # 有案件按下
                if event.key == pg.K_ESCAPE:
                    self.quit()

    def show_start_screen(self):
        pass

    def show_go_screen(self):
        pass

if __name__ == '__main__':
    g = Game()
    # g.show_start_screen()
    while True:
        g.new()
        g.run()
        g.show_go_screen()