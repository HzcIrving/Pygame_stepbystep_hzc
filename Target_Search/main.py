# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

from env import MAZE
from Q_L import QLearningTable

def main():
    env = MAZE()
    # 0,1,2,3
    RL = QLearningTable(actions=list(range(env.n_actions)))

    for episode in range(500):
        # 初始化观察值
        observation = env.reset()

        while True:
            # 刷新env
            env.render()

            # 选择action
            action = RL.choose_action(str(observation))

            # RL take action，获得next_observation与reward
            observation_,reward,done = env.step(action)

            # RL从transition中学习
            RL.learn(str(observation),action,reward,str(observation_))

            # 更新observation
            observation = observation_

            if done:
                break

    print('game over')
    env.destroy()

if __name__ == '__main__':
    main()

