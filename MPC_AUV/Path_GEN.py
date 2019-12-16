# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

from MPC_auv import *
from utils import *
from CubicSpline import *

def get_switch_back_course(dl):
    ax = [0.0,30.0,6.0,20.0,35.0]
    ay = [0.0,0.0,20.0,35.0,20.0]
    goal = [ax[-1],ay[-1]]
    cx,cy,cpsi,ck,s = calc_spline_course(ax,ay,ds=dl)

    # ax = [35.0,10.0,0.0,0.0]
    # ay = [20.0,30.0,5.0,0.0]
    # cx2,cy2,cpsi2,ck2,s2 = calc_spline_course(ax,ay,ds=dl)
    # cpsi2 = [i-math.pi for i in cpsi2] # 修正反方向
    # cx.extend(cx2)
    # cy.extend(cy2)
    # cpsi.extend(cpsi2)
    # ck.extend(ck2)

    return cx,cy,cpsi,ck,goal

def get_straight_course(dl):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal = [ax[-1],ay[-1]]
    cx, cy, cpsi, ck, s = calc_spline_course(
        ax, ay, ds=dl)
    return cx,cy,cpsi,ck,goal

def get_straight_course2(dl):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    goal = [ax[-1],ay[-1]]

    cx, cy, cyaw, ck, s = calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck, goal