import matplotlib.pyplot as plt
import numpy as np
from param import param
P = param()

class vtolAnimation:
    def __init__(self):
        self.flagInit = False
        self.fig, self.ax = plt.subplots()
        self.handle = []

        plt.axis([-6.0*P.L, 6.0*P.L, -6.0*P.L, 6.0*P.L])
        self.ax.set_aspect('equal')

    def update(self,state):
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)

        #Define coordinates of all
        lineX = [z+P.L*np.cos(theta), z-P.L*np.cos(theta)]
        lineY = [h+P.L*np.sin(theta), h-P.L*np.sin(theta)]
        # lineX,lineY = P.R(lineX,lineY,theta)

        boxX = [z-P.boxWidth,z-P.boxWidth,z+P.boxWidth,z+P.boxWidth]
        boxY = [h-P.boxWidth,h+P.boxWidth,h+P.boxWidth,h-P.boxWidth]
        boxX, boxY = P.R(boxX,boxY,state)

        pointsX = [z+P.L*np.cos(theta), z-P.L*np.cos(theta)]
        pointsY = [h+P.L*np.sin(theta), h-P.L*np.sin(theta)]
        # pointsX,pointsY = P.R(pointsX,pointsY,theta)

        if self.flagInit == False:
            self.flagInit = True
            line, = self.ax.plot(lineX,lineY)
            box, = self.ax.fill(boxX,boxY)
            fans = self.ax.scatter(pointsX,pointsY,color='#1f77b4',s=16)

            self.handle.append(line)
            self.handle.append(box)
            self.handle.append(fans)

        if self.flagInit == True:
            self.handle[0].set_xdata(lineX)
            self.handle[0].set_ydata(lineY)

            box_cords = list(zip(boxX, boxY))
            self.handle[1].set_xy(box_cords)

            points_cords = list(zip(pointsX,pointsY))
            self.handle[2].set_offsets(points_cords)

        plt.draw()