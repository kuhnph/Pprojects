import matplotlib.pyplot as plt
import numpy as np

class quadRotorFuncPlots:
    def __init__(self):
        #initialize figures/axis
        self.fig1, self.axs1 = plt.subplots(3,1)

        #set x/y labels
        for i in range(len(self.axs1)): self.axs1[i].set_xlabel('time')
        yLabels = ['x [m]', 'y [m]', 'z [m]']
        for i in range(len(yLabels)): self.axs1[i].set_ylabel(yLabels[i])

        #setup handle for storing and updating position data
        self.xData, = self.axs1[0].plot([],[], label='X position')
        self.yData, = self.axs1[1].plot([],[], label='Y position')
        self.zData, = self.axs1[2].plot([],[], label='Z position')

        #setup handle for storing and updating Commanded data
        self.xRefData, = self.axs1[0].plot([],[], label='X Commanded')
        self.yRefData, = self.axs1[1].plot([],[], label='Y Commanded')
        self.zRefData, = self.axs1[2].plot([],[], label='Z Commanded')

        #add legend
        for ax in self.axs1: 
            ax.legend()
            ax.grid(True)

        plt.ion()
        plt.show()

    def update(self, i, timeHistory, stateHistory, refVars):
        # i = current frame index (FuncAnimation calls this)
        # update data up to frame i
        t = timeHistory[:i]
        x = stateHistory[:i, 0]
        y = stateHistory[:i, 1]
        z = stateHistory[:i, 2]
        xRef = refVars[:i, 0]
        yRef = refVars[:i, 1]
        zRef = refVars[:i, 2]

        # update the line data
        self.xData.set_data(t, x)
        self.yData.set_data(t, y)
        self.zData.set_data(t, z)
        self.xRefData.set_data(t,xRef)
        self.yRefData.set_data(t,yRef)
        self.zRefData.set_data(t,zRef)

        #adjust vertical axis limits
        if i == 0: 
            for ax in self.axs1:
                ax.set_xlim(0,timeHistory[-1])
            self.axs1[0].set_ylim(min(stateHistory[:,0])*1.05, max(stateHistory[:,0])*1.05)
            self.axs1[1].set_ylim(min(stateHistory[:,1])*1.05, max(stateHistory[:,1])*1.05)
            self.axs1[2].set_ylim(min(stateHistory[:,2])*1.05, max(stateHistory[:,2])*1.05)
        # for ax in self.axs1:
        #     ax.relim()
        #     ax.autoscale_view(scalex=False, scaley=True)

        # Redraw
        self.fig1.canvas.draw()
        self.fig1.canvas.flush_events()

        # return artists that have changed
        return [self.xData, self.yData, self.zData]

F = funcPlots()
dat = np.load('sim_states.npz')
time = dat['time']
stateVars=dat['stateVars']
refVars=dat['refVars']

#test loop
for i in range(len(time)):
    F.update(i, time, stateVars, refVars)

print('done')
