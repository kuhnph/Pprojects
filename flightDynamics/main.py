# Add the absolute path to the src folder
import sys
import os
src_path = os.path.join(os.path.dirname(__file__), "src")
sys.path.append(src_path)

import numpy as np
from params import params
from dynamics import dynamics
from plotting import plotting
from animation import animation

#Initialize Classes
D = dynamics()
P = params()
Pl = plotting()
A = animation(P.state0)
timeSteps = int(P.T_end/P.Ts)
time = 0
u_star = P.u_star #Create trim states

#Set up storage for states
stateStorage = np.zeros((timeSteps,len(D.state[:,0])+1))
FaMStorage = np.zeros((timeSteps,7))

for i in range(timeSteps):

    #Update the dynamics
    D.update(u_star)

    #update the animation
    if i % P.plot_delimination == 0:
        A.update(D.state, time)
        
        #Kill the animation if I want
        if A.killFlag:
            break
    
    #Store state variables
    stateStorage[i] = np.concatenate((D.state.flatten(), [time]))
    FaMStorage[i] = np.concatenate((np.concatenate((P.R_body_to_inertial(D.state.item(6),D.state.item(7),D.state.item(8))@D.F.flatten(),D.M.flatten())), [time]))
    time+=P.Ts

print('Sim Ended')
Pl.staticPlotState(stateStorage[0:i])
Pl.staticPlotFaM(FaMStorage[0:i])