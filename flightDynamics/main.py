# Add the absolute path to the src folder
import sys
import os
src_path = os.path.join(os.path.dirname(__file__), "src")
sys.path.append(src_path)

import numpy as np
from sim.params import params
from sim.dynamics import dynamics
from viewer.plotting import plotting
from viewer.animation import animation
from control.trimCalculation import main as trim


#Initialize Classes
D = dynamics()
P = params()
Pl = plotting()
A = animation(P.state0)
timeSteps = int(P.T_end/P.Ts)
time = 0

#Create states for trim
u_star, state0 = trim()
D.state=state0

#Set up storage for states
stateHistory = np.zeros((timeSteps,len(D.state[:,0])))
FaMHistory = np.zeros((timeSteps,6))
timeHistory = np.zeros((timeSteps,1))

#Flag and setup for video saving
saveVideo = False
if saveVideo: 
    import matplotlib.animation as animation
    from funcAnimate import funcAnimation
    F = funcAnimation()

for i in range(timeSteps):

    #Update the dynamics
    D.update(u_star)

    #update the animation
    if i % P.plot_delimination == 0 and not saveVideo:
        A.update(D.state, time)
        
        #Kill the animation if I want
        if A.killFlag:
            break
    
    #Store state variables
    stateHistory[i] = D.state.flatten()
    FaMHistory[i] = np.concatenate((P.R_body_to_inertial(D.state.item(6),D.state.item(7),D.state.item(8))@D.F.flatten(),D.M.flatten()))
    timeHistory[i] = time
    time+=P.Ts

if saveVideo:
    print('frame count:', timeSteps)
    vehicleMovie = animation.FuncAnimation(F.fig, F.update, int(i), fargs=(stateHistory,timeHistory,),  interval=1, blit=False)
    print('Saving Vehicle Movie...')
    vehicleMovie.save("results/VehicleMovie.gif", writer=animation.PillowWriter(fps=30))
    print('Sim Ended')

#Flag for showing plots at the end
PLOTS = False
if PLOTS:
    Pl.staticPlotState(stateHistory[0:i], timeHistory[0:i])
    Pl.staticPlotFaM(FaMHistory[0:i], timeHistory[0:i])