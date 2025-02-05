from animation2 import vtolAnimation
from dynamics import dynamics
from param import param
from control import controller
from plotter import livePlot, staticPlot
import numpy as np
import matplotlib.pyplot as plt


def main(PLOTFLAG=False,ANIMATEFLAG=False):

    #Initialize classes
    P = param()
    D = dynamics()
    C = controller(type="FSFB")
    if ANIMATEFLAG: A = vtolAnimation()
    if PLOTFLAG: SP = staticPlot()

    #initialize time variables
    t_end = P.t_end
    time = 0
    simulation_step = P.simTimeStep
    animation_step = P.animationTimeStep

    #initialize state history
    stateHistory = np.zeros((int(t_end/P.simTimeStep),D.state.shape[0]+3))
    
    for i in range(int(t_end/P.simTimeStep)):

        #update control input
        u = C.update(P.Zr,P.Hr,D.state)

        #update dynamics
        D.update(u)

        #update animation
        if ANIMATEFLAG: A.update(D.state,time)

        #propogate time
        time += simulation_step

        #update state history
        stateHistory[i][0:6] = D.state.T[0]
        stateHistory[i][6] = P.Zr
        stateHistory[i][7] = P.Hr
        stateHistory[i][-1]=time

    #produce plot
    if PLOTFLAG: SP.makePlot(stateHistory)

    return stateHistory


if __name__ == "__main__":
    main(ANIMATEFLAG=True,PLOTFLAG=True)