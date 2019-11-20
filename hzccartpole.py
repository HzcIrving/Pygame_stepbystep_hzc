# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

import logging
import math
import gym
from gym.utils import seeding
from gym import spaces
import numpy as np

"""
Description:
    A pole is attached by an un-actuated joint to a cart, which moves along a frictionless track. The pendulum starts upright, and the goal is to prevent it from falling over by increasing and reducing the cart's velocity.
Source:
    This environment corresponds to the version of the cart-pole problem described by Barto, Sutton, and Anderson
Observation: 
    Type: Box(4)
    Num	Observation                 Min         Max
    0	Cart Position             -4.8            4.8
    1	Cart Velocity             -Inf            Inf
    2	Pole Angle                 -24 deg        24 deg
    3	Pole Velocity At Tip      -Inf            Inf

Actions:
    Type: Discrete(2)
    Num	Action
    0	Push cart to the left
    1	Push cart to the right

    Note: The amount the velocity that is reduced or increased is not fixed; it depends on the angle the pole is pointing. This is because the center of gravity of the pole increases the amount of energy needed to move the cart underneath it
Reward:
    Reward is 1 for every step taken, including the termination step
Starting State:
    All observations are assigned a uniform random value in [-0.05..0.05]
Episode Termination:
    Pole Angle is more than 12 degrees
    Cart Position is more than 2.4 (center of the cart reaches the edge of the display)
    Episode length is greater than 200
    Solved Requirements
    Considered solved when the average reward is greater than or equal to 195.0 over 100 consecutive trials.
"""

# 日志项
logger = logging.getLogger(__name__)

# 继承gym.Env
class CartPole_hzc(gym.Env):
    metadata = {
        'render.modes':['human','rgb_array'],
        'video.frame_per_second':50 # 每秒帧数
    }

    def __init__(self):
        # 变量声明
        self.gravity = 9.8 # 重力加速度
        self.masscart = 1.0 # 小车质量
        self.masspole = 1.0 # 摆的质量
        self.total_mass = (self.masspole+self.masscart) # 总质量
        self.length = 0.5 # actually half the pole's length  #摆的半长，即摆的转动中心到摆的质心的距离
        self.polemass_length = (self.masspole * self.length) #摆的质量长
        self.force_mag = 10.0                                #外力的振幅
        self.tau = 0.02  # seconds between state updates     #更新时间步

        self.theta_threshold_radians  = 12*2*math.pi/360 # 摆角最大范围（弧度)
        self.x_thresold = 2.4 # x 方向最大移动范围

        # 设置范围最大在2*theta_threshold_radians，观察到失败时仍然在范围内
        high = np.array([
            self.x_thresold*2,
            np.finfo(np.float32).max,
            self.theta_threshold_radians*2,
            np.finfo(np.float32).max
            ])

        self.action_space = spaces.Discrete(2) #小车倒立摆的动作空间为2维离散空间
        # 观测空间为（x,dx,theta,dtheta）区间为（（-24，24），（-2.4，2.4））
        self.observation_space = spaces.Box(-high,high)

        self.seed()
        self.viewer = None
        self.state = None

        self.steps_beyond_done = None

    def _seed(self,seed=None):
        """
        设置随机种子seed
        :return:
        """
        self.np_random,seed = seeding.np_random(seed)
        return [seed]

    def _step(self,action):
        """
        动力学相关
        :param action:输入动作
        :return: state,reward,done,information
        """
        # 判断输入动作是否合理
        assert self.action_space.contains(action), "%r (%s)不合理！"%(action,type(action))
        state = self.state

        x,x_dot,theta,theta_dot = state

        # 如果action = 1 力的方向→推小车
        # 如果action = -1 力的方向←推小车
        force = self.force_mag if action == 1 else -self.force_mag

        # 三角函数
        costheta = math.cos(theta)
        sintheta = math.sin(theta)

        # 动力学方程
        # x'' = [（F+mlθ'^2sinθ) - mlθ''cosθ]/(m+M)
        temp = (force+self.polemass_length*theta_dot*theta_dot*sintheta)/self.total_mass
        # 求 θ''
        thetaacc = (self.gravity * sintheta - costheta* temp) / (self.length * (4.0/3.0 - self.masspole * costheta * costheta / self.total_mass))
        xacc = temp - self.polemass_length*thetaacc*costheta/self.total_mass

        # 积分求下一步状态
        x = x + self.tau*x_dot # self.tau是更新率
        x_dot = x_dot + self.tau*xacc
        theta = theta + self.tau*theta_dot
        theta_dot = theta_dot + self.tau*thetaacc

        self.state = (x,x_dot,theta,theta_dot)

        done = x < -self.x_thresold or x > self.x_thresold or \
               theta < -self.theta_threshold_radians or theta > self.theta_threshold_radians
        done = bool(done) #转化为布尔型

        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None :
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warning("You are calling 'step()' even though this environment has already returned done = True. You should always call 'reset()' once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0
        return np.array(self.state),reward,done,{}

    def _reset(self):
        self.state = self.np_random.uniform(low=-0.05,high=0.05,size=(4,))
        self.steps_beyond_done = None
        return np.array(self.state)

    def render(self,model='human',close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer  = None
            return

        screen_width = 800
        screen_height = 600

        world_width = self.x_thresold*2
        scale = screen_width/world_width # 尺度
        carty = 100 # Cart 最远距离
        polewidth = 10.0 # 杆粗
        pole_len = scale*1.0 # 杆长
        cartwidth = 50.0 # 小车长
        cartheight = 30.0 # 小车高

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(screen_width,screen_height)
            # 创建一台车--------------------------------------------------
            # 坐标在中心
            # l=left,r=right,t=top,b=bottom
            l,r,t,b = -cartwidth/2,cartwidth/2,cartheight/2,-cartheight/2
            axleoffsetting = cartheight/4.0
            # 填充几何体
            cart = rendering.FilledPolygon([(l,b),(l,t),(r,t),(r,b)])
            # 添加台车，转换矩阵属性
            self.carttrans = rendering.Transform()
            cart.add_attr(self.carttrans)
            # 加入几何体车
            self.viewer.add_geom(cart)

            # 创建摆杆--------------------------------------------------
            l,r,t,b = -polewidth/2,polewidth/2,pole_len-polewidth/2,-polewidth/2
            pole = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
            pole.set_color(.8,.6,.4) # 填充颜色
            # 添加摆杆转换矩阵属性
            self.poletrans = rendering.Transform(translation=(0,axleoffsetting))
            pole.add_attr(self.poletrans)
            pole.add_attr(self.carttrans)
            # 加入几何体
            self.viewer.add_geom(pole)

            # 创建摆杆和车之间的链接
            self.axle = rendering.make_circle(polewidth/2)
            self.axle.add_attr(self.poletrans)
            self.axle.add_attr(self.carttrans)
            self.axle.set_color(.5,.5,.8)
            self.viewer.add_geom(self.axle)

            # 创建台车来回滑动的导轨
            self.track = rendering.Line((0,carty),(screen_width,carty)) # 起点、终点
            self.track.set_color(0,0,0)
            self.viewer.add_geom(self.track)

        if self.state is None: return None

        x = self.state # state x,dx,theta,dtheta
        cartx = x[0]*scale+screen_width/2.0 # 从中间开始
        # 设置平移属性
        self.carttrans.set_translation(cartx,carty) # 最远carty
        self.carttrans.set_rotation(-x[2])

        return self.viewer.render(return_rgb_array = mode=='rgb_array')

    def _close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None


