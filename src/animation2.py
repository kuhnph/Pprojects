import matplotlib.pyplot as plt
import numpy as np
from param import param
P = param()


def create_figure(xmin=-10,xmax=10,ymin=-10,ymax=10,height=12,width=12):
    fig, ax = plt.subplots()
    ax.set_xlim(xmin,xmax)
    ax.set_ylim(ymin,ymax)
    fig.canvas.draw()
    fig.set_size_inches(height,width)

    # cache the background
    plt.pause(.0001)
    bg = fig.canvas.copy_from_bbox(ax.bbox)
    plt.show(block=False)
    return fig, ax, bg

class vtolAnimation:
    def __init__(self):
        self.flagInit = False
        self.fig,self.ax,self.bg = create_figure()
        self.handle = []

        # plt.axis([-6.0*P.L, 6.0*P.L, -6.0*P.L, 6.0*P.L])
        self.ax.set_aspect('equal')

    #MAKE
    def make(self):
        self.fig.canvas.restore_region(self.bg)
        for h in self.handle:
            self.ax.draw_artist(h)
        self.fig.canvas.blit(self.ax.bbox)  
        self.fig.canvas.flush_events()

    def update(self,state,T):
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)

        #Define coordinates of all shapes (body, arms, fans)
        lineX = [z+P.L*np.cos(theta), z-P.L*np.cos(theta)]
        lineY = [h+P.L*np.sin(theta), h-P.L*np.sin(theta)]
        # print(h+P.L*np.sin(theta))
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
            text = self.ax.text(0.8,0.5, '')

            self.handle.append(line)
            self.handle.append(box)
            self.handle.append(fans)
            self.handle.append(text)

        if self.flagInit == True:
            self.handle[0].set_xdata(lineX)
            self.handle[0].set_ydata(lineY)

            box_cords = list(zip(boxX, boxY))
            self.handle[1].set_xy(box_cords)

            points_cords = list(zip(pointsX,pointsY))
            self.handle[2].set_offsets(points_cords)

            tx = f"T={format(T,'2f')}"
            self.handle[3].set_text(tx)
        
        self.make()