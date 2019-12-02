# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

"""
迷宫寻宝env，探索一般自写RL环境的一般形式

###
黑色矩形：地狱 --> [Reward=-1]
黄色圆形：天堂 --> [Reward=+1]
其他状态：地面 --> [Reward= 0]
红色矩形: 探索者...
"""

import numpy as np
import time
import sys
if sys.version_info.major == 2:
    import Tkinter as tk
else:
    import tkinter as tk

UNIT = 5
MAZE_H = 100
MAZE_W = 100

class MAZE(tk.Tk,object):
    """
    迷宫环境...厉害了！
    """
    def __init__(self):
        super(MAZE,self).__init__()
        self.action_space = ['u','d','l','r'] # 上下左右
        self.n_actions = len(self.action_space) # action行为维度
        self.title('maze')
        self.geometry('{0}x{1}'.format(MAZE_H*UNIT,MAZE_H*UNIT))
        self._build_maze()

    def _build_maze(self):
        """搭建MAZE环境"""
        self.canvas = tk.Canvas(self,bg='gray',height=MAZE_H*UNIT,width=MAZE_W*UNIT) # 画布
        # create grids
        for c in range(0,MAZE_W*UNIT,UNIT):
            x0,y0,x1,y1 = c,0,c,MAZE_H*UNIT
            # 画竖线
            self.canvas.create_line(x0,y0,x1,y1) # 起始点、终点
        for r in range(0,MAZE_H*UNIT,UNIT):
            x0,y0,x1,y1 = 0,r,MAZE_W*UNIT,r
            self.canvas.create_line(x0,y0,x1,y1) # 画横线

        # create origin
        origin = np.array([20,20]) # 第一个grid(尺寸40)的中心'
        paradise = np.array([497.5,497.5])

        # hell
        hell1_center = origin+np.array([UNIT*4,UNIT]) # hell的中心坐标
        self.hell1 = self.canvas.create_rectangle(hell1_center[0]-15,hell1_center[1]-15,
                                                  hell1_center[0]+15,hell1_center[1]+15,
                                                  fill = 'white')  # 起始点、终点
        hell2_center = origin+np.array([UNIT,4*UNIT])
        self.hell2 = self.canvas.create_rectangle(hell2_center[0]-15,hell2_center[1]-15,
                                                  hell2_center[0]+15,hell2_center[1]+15,
                                                  fill = 'white')

        # 最终奖励标志
        # oval_center = origin+UNIT*4
        oval_center = paradise
        self.oval = self.canvas.create_oval(oval_center[0]-2.5,oval_center[1]-2.5,
                                            oval_center[0]+2.5,oval_center[1]+2.5,
                                            fill = 'yellow')

        # explorer 探索者
        self.rect = self.canvas.create_rectangle(origin[0]-15,origin[1]-15,
                                                 origin[0]+15,origin[1]+15,
                                                 fill = 'red')

        # pack all
        self.canvas.pack()

    def reset(self):
        """重置初始化"""
        self.update()
        time.sleep(0.5)
        self.canvas.delete(self.rect) # 回到原点
        origin = np.array([20,20])
        self.rect = self.canvas.create_rectangle(origin[0]-15,origin[1]-15,
                                                 origin[0]+15,origin[1]+15,
                                                 fill = 'red')
        # 返回rect的坐标
        return self.canvas.coords(self.rect)

    def step(self,action):
        """执行action"""
        s = self.canvas.coords(self.rect) # 获得探索者坐标
        base_action = np.array([0,0])
        if action == 0: #up
            if s[1] > UNIT:
                base_action[1] -= UNIT
        elif action == 1: # down
            if s[1]<(MAZE_H - 1)*UNIT:
                base_action[1] += UNIT
        elif action == 2: # right
            if s[0]<(MAZE_W - 1)*UNIT: # 右边界
                base_action[0] += UNIT
        elif action == 3: # left
            if s[0]>UNIT: # 左边界
                base_action[0] -= UNIT

        self.canvas.move(self.rect,base_action[0],base_action[1]) # 移动探索者

        s_ = self.canvas.coords(self.rect) # next_state

        # reward function
        if s_ == self.canvas.coords(self.oval): # 若当前位置在天堂处
            reward = 1
            done = True
            s_ = 'terminal'
        elif s_ in [self.canvas.coords(self.hell1),self.canvas.coords(self.hell2)]: # 跑到地狱去了
            reward = -1
            done = True
            s_ = 'terminal'
        else:
            reward = 0
            done = False

        return s_, reward,done

    def render(self):
        """使能"""
        time.sleep(0.01)
        self.update()

def update(env):
    for t in range(10):
        s = env.reset()
        while True:
            env.render()
            a = 1
            s,r,done = env.step(a)
            if done or s:
                break

# if __name__ == '__main__':
#     env = MAZE()
#     # env.after(100,update())
#     # env.mainloop()
#     s = env.reset()
#     while True:
#         env.render()
#         a = 1
#         s,r,done = env.step(a)
#         if s[1]==(MAZE_H-2)*UNIT:
#             s = env.reset()



