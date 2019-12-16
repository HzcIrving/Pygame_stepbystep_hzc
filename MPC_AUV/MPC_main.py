# -*- coding=utf-8 -*-
# @ Author:HeZichen
# @ Email:irvingChen1518@gmail.com  
# @ Python script

# from LQR_Steering_Control import *

import math
import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg as la
import CubicSpline
from MPC_auv import *
from utils import  *
from Path_GEN import *

show_animation = True

dl = 1.0 # 刻度[m]

# cx,cy,cpsi,ck,goal = get_switch_back_course(dl)
# cx,cy,cpsi,ck,goal = get_straight_course(dl)
cx,cy,cpsi,ck,goal = get_straight_course2(dl)

plt.plot(cx,cy)
plt.show()
# print(cx)
# print(cy)
print(goal)

sp = calc_speed_profile(cx,cy,cpsi,TARGET_SPEED_U,TARGET_SPEED_V)
plt.plot(sp[0])
plt.plot(sp[1])
plt.show()

initial_state = STATE(X=cx[0],Y=cy[0],psi=cpsi[0])

t,x,y,psi,u,v,r = do_simulation(cx,cy,cpsi,ck,dl,initial_state,sp)

if show_animation:  # pragma: no cover
    plt.close("all")
    plt.subplot(2,3,1)
    plt.plot(cx, cy, "-r", label="spline")
    plt.plot(x, y, "-g", label="tracking")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplot(2,3,2)
    # plt.plot(t, v, "-r", label="speed"
    plt.plot(cpsi,label='psi_d')
    plt.plot(psi[1:],label='psi')
    plt.grid(True)
    # plt.xlabel("Time [s]")
    # plt.ylabel("Speed [kmh]")

    plt.subplot(2,3,3)
    plt.plot(sp[0],label='u_d')
    plt.plot(u,label='u')
    plt.grid(True)

    plt.subplot(2,3,4)
    plt.plot(sp[1],label='v_d')
    plt.plot(v,label='v')
    plt.grid(True)


    plt.show()









