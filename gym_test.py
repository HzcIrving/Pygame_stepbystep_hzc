# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import gym
# from hzccartpole import CartPole_hzc
import time
# env = gym.make('CartPole-v0')
# env = CartPoleEnv()
env = gym.make('hzcPole-v0')
obs = env.reset()


count = 0
while True:
    action = env.action_space.sample()

    obs, reward, done, info = env.step(action)
    if done:
        break
    env.render()
    count += 1
    # time.sleep(0.2)
    # if count > 100:
    #     break
env.close()
