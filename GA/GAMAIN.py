from animation2 import vtolAnimation
from dynamics import dynamics
from param import param
from control import controller
from plotter import livePlot, staticPlot
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def tune(kp_z=0,kd_z=0,kp_theta=0,kd_theta=0,kp_h=0,kd_h=0,tuneH=False,tuneZ=False,PLOTFLAG=False,ANIMATEFLAG=False):
    P = param()
    D = dynamics()
    if ANIMATEFLAG:A = vtolAnimation()
    if PLOTFLAG:SP = staticPlot()
    if tuneH:
        C = controller(kp_z=P.kp_z,kd_z=P.kd_z, kp_theta=P.kp_theta,kd_theta=P.kd_theta, kp_h=kp_h,kd_h=kd_h)
    elif tuneZ:
        C = controller(kp_z=kp_z,kd_z=kd_z, kp_theta=kp_theta,kd_theta=kd_theta, kp_h=P.kp_h,kd_h=P.kd_h)
    else:
        C = controller(kp_z,kd_z, kp_theta,kd_theta, kp_h,kd_h)

    t_end = P.t_end
    simulation_step = P.simTimeStep
    animation_step = P.animationTimeStep
    stateHistory = np.zeros((int(t_end/P.simTimeStep),D.state.shape[0]+3))
    U_H = P.U_H
    U_Z = P.U_Z
    time = 0
    for i in range(int(t_end/P.simTimeStep)):

        #update control input
        u = C.update(U_H,U_Z,D.state)

        #update dynamics
        D.update(u)

        #update animation
        if ANIMATEFLAG: A.update(D.state,time)

        #propogate time
        time += simulation_step


        stateHistory[i][0:6] = D.state.T[0]
        stateHistory[i][6] = U_H
        stateHistory[i][7] = U_Z
        stateHistory[i][-1]=time
    df = pd.DataFrame(data=stateHistory, index=stateHistory.T[-1],
                      columns='z,h,theta,zDot,hDot,thetaDot,U_H,U_Z,time'.split(','))
    
    if PLOTFLAG:
        SP.makePlot(stateHistory)
    return df

if __name__ == "__main__":
    P = param()
    df, u_lat, u_long = tune(kp_z=P.kp_z,kd_z=P.kd_z, kp_theta=P.kp_theta,kd_theta=P.kd_theta, kp_h=P.kp_h,kd_h=P.kd_h, PLOTFLAG=True,ANIMATEFLAG=True)