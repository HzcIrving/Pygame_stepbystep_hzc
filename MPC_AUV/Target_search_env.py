# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import matplotlib.pyplot as plt

DT = 0.2
TARGET_X = 80
TARGET_Y = 80
TARGET_PSI = np.deg2rad(45)

class AUV(object):
  def __init__(self,target_x,target_y,target_psi):
    self.state = np.zeros((3,))
    self.target_x = target_x
    self.target_y = target_y
    self.target_psi = target_psi

    self.done = False

    self.u = 10
    self.v = 10
    # self.max_speed_u = 55.0*0.7/3.6
    # self.max_speed_v = 55.0*0.7/3.6
    # self.min_speed_u = -20.0*0.7/3.6 # m/s
    # self.min_speed_v = -20.0*0.7/3.6 # m/s
    #
    # self.max_r = np.deg2rad(30.0) # rad/s
    # self.min_r = -np.deg2rad(30.0) # rad/s
    #
    # self.max_a = np.array([self.max_speed_u,self.max_speed_v,self.max_r])
    # self.min_a = np.array([self.min_speed_u,self.min_speed_v,self.min_r])

    self.action_space = spaces.Discrete(3)

    # self.action_space = spaces.Box(low=-self.max_a,high=self.max_a,dtype=np.float32)
    self.observation_space = spaces.Box(low=np.array([0,0,0]),high=np.array([100,100,np.deg2rad(359)]))

    self.seed()

  def seed(self,seed=None):
    self.np_random,seed=seeding.np_random(seed)
    return [seed]

  def step(self,u):

    dt = DT

    x,y,psi = self.state

    if u == 0:
      r = np.deg2rad(20)
    elif u == 1:
      r = np.deg2rad(-20)
    elif u == 2:
      r = np.deg2rad(0)
        # elif
    """限制"""
    # u[0] = np.clip(u[0],-self.max_speed_u,self.max_speed_u)
    # u[1] = np.clip(u[1],-self.max_speed_v,self.max_speed_v)
    # u[2] = np.clip(u[2],-self.max_r,self.max_r)
    # u[0] = np.clip(u[0],-self.max_speed_u,self.max_speed_u)
    # u[1] = np.clip(u[1],-self.max_speed_v,self.max_speed_v)
    # u[2] = np.clip(u[2],-self.max_r,self.max_r)

    self.last_u = u  # for rendering

    new_x = x + dt*(np.cos(psi)*self.u-np.sin(psi)*self.v)
    new_y = y + dt*(np.sin(psi)*self.u+np.cos(psi)*self.v)
    new_psi = psi+dt*r

    d = np.sqrt((new_x-self.target_x)**2+(new_y-self.target_y)**2)

    if d <= 0.01 and new_psi-TARGET_PSI<0.1:
      reward = 100
      self.done = True
    else:
      reward = 100/d

    self.state = np.array([new_x,new_y,new_psi])

    return self._get_obs(),-reward,self.done,{}

  def reset(self):
    self.state = np.zeros((3,))
    self.last_u = None
    return self._get_obs()

  def _get_obs(self):
    x,y,psi = self.state
    return np.array([x,y,psi])

"""
Q Learning算法
"""

import numpy as np
import pandas as pd

class QLearningTable:
    def __init__(self,actions,learning_rate=0.01,reward_decay=0.9,e_greedy=0.9):
        self.actions = actions
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon = e_greedy
        # 有action列
        self.q_table = pd.DataFrame(columns=self.actions,dtype=np.float64)

    def choose_action(self,observation):
        self.check_state_exist(observation)
        # action selection_exploration and exploitation
        if np.random.uniform() < self.epsilon:
            state_action = self.q_table.loc[observation,:]
            # action = np.argmax(state_action)
            action = np.random.choice(state_action[state_action == np.max(state_action)].index)
        else:
            action = np.random.choice(self.actions) # 随机 探索
        return action

    def learn(self,s,a,r,s_):
        self.check_state_exist(s_)
        q_predict = self.q_table.loc[s,a]
        # exploration
        # if s_!='terminal':
        if s_[0] == TARGET_X and s_[1] == TARGET_Y:
            # 最大Q值
            q_target = r + self.gamma*self.q_table.loc[s_,:].max() # 非terminal的情况
        else:
            q_target = r # terminal情况
        self.q_table.loc[s,a] += self.lr*(q_target-q_predict) # update

    def check_state_exist(self,state):
        """若状态不在Q表内，则添加进Q表"""
        # pass
        if state not in self.q_table.index:
            # 添加新的state to Q表
            self.q_table = self.q_table.append(
                pd.Series(
                    [0]*len(self.actions),
                    index = self.q_table.columns,
                    name = state
                )
            )

def main():
  env = AUV(TARGET_X,TARGET_Y,TARGET_PSI)
  # 0,1,2,3
  RL = QLearningTable(actions=list(range(3)))
  x = []
  y = []
  psi = []
  rd = 0
  acc_rd = []

  for episode in range(500):
    # 初始化观察值
    observation = env.reset()

    while True:
      # 刷新env
      # env.render()

      # 选择action
      action = RL.choose_action(str(observation))

      # RL take action，获得next_observation与reward
      observation_, reward, done,_ = env.step(action)

      x.append(observation_[0])
      y.append(observation_[1])
      psi.append(observation_[2])

      # rd += reward
      acc_rd.append(reward)

      # RL从transition中学习
      RL.learn(observation, action, reward, observation_)

      # 更新observation
      observation = observation_

      if done:
        break

      x = []
      y = []
      z = []

  plt.subplots()
  plt.plot(TARGET_X,TARGET_Y,'-og')
  plt.plot(x,y,'-r')
  plt.grid(True)
  plt.legend()

  plt.subplots()
  plt.plot(acc_rd)
  plt.grid(True)

  print('game over')
  # env.destroy()


if __name__ == '__main__':
  main()