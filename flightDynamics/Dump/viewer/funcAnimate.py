# Add the absolute path to the src folder
import sys
import os
src_path = os.path.join(os.path.dirname(__file__), "src").replace('WIP', "flightDynamics")
sys.path.append(src_path)

import numpy as np
import matplotlib.pyplot as plt
from drawPlane import drawPlane
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation


class funcAnimation:
    def __init__(self):
        # Create 3D plot
        self.fig = plt.figure(figsize=(8,8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title('SIMULATION')

        # Time text (top-left corner)
        self.time_text = self.ax.text2D(0.05, 0.95, '', transform=self.ax.transAxes)

        # Label axes
        self.ax.set_xlabel('North [m]')
        self.ax.set_ylabel('East [m]')
        self.ax.set_zlabel('Down [m]')

        # Set axis limits
        self.ax.set_xlim([-200, 200])  # North
        self.ax.set_ylim([-200, 200])  # East
        self.ax.set_zlim([-200, 200])   # Down (remember, positive down in NED)

        # Set equal aspect for better visualization
        self.ax.set_box_aspect([1,1,1])
    
        self.polyHandle = []

    def update(self,i, stateHistory, timeHistory):
        time = timeHistory.item(i)
        pn = stateHistory[:,0][i]
        pe = stateHistory[:,1][i]
        pd = stateHistory[:,2][i]
        phi = stateHistory[:,6][i]
        theta = stateHistory[:,7][i]
        psi = stateHistory[:,8][i]

        #Set time text
        self.time_text.set_text(f"Time: {time:.2f}s")

        #clear polygons
        for poly in self.polyHandle:
            poly.remove()
        self.polyHandle = []

        #Get plane faces in inertial frame
        F = drawPlane(pn, pe, pd, phi, theta, psi)

        #Draw faces and store for removal
        for face in F:
            poly = Poly3DCollection([face], facecolors='lightblue', linewidth=1, edgecolors='black')
            self.ax.add_collection3d(poly)
            self.polyHandle.append(poly)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()



