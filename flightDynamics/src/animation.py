import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import Button
from params import params
from drawPlane import drawPlane

P = params()
class animation:
    def __init__(self,state):
        # Create 3D plot
        self.fig = plt.figure(figsize=(12,12))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        #Kill button
        plt.subplots_adjust(bottom=0.2)
        kill_ax = plt.axes([0.8, 0.05, 0.1, 0.075])
        self.kill_button = Button(kill_ax, 'Kill', color='red', hovercolor='salmon')
        self.killFlag = False

        # Time text (top-left corner)
        self.time_text = self.ax.text2D(0.05, 0.95, '', transform=self.ax.transAxes)

        # Label axes
        self.ax.set_xlabel('North [m]')
        self.ax.set_ylabel('East [m]')
        self.ax.set_zlabel('Down [m]')

        # Set axis limits
        self.ax.set_xlim([-100, 100])  # North
        self.ax.set_ylim([-100, 100])  # East
        self.ax.set_zlim([-100, 100])   # Down (remember, positive down in NED)

        # Set equal aspect for better visualization
        self.ax.set_box_aspect([1,1,1])


        pn = state.item(0)
        pe = state.item(1)
        pd = state.item(2)
        phi = state.item(6)
        theta = state.item(7)
        psi = state.item(8)
        self.poly_handle = []

    def kill(self,event):
        self.killFlag = True
        plt.close()

    def draw(self, state, t=0):
        self.kill_button.on_clicked(self.kill)

        # Update time display
        self.time_text.set_text(f"Time: {t:.2f} s")

        # Clear previous polygons
        for poly in self.poly_handle:
            poly.remove()
        self.poly_handle = []

        # Extract state variables
        pn = state.item(0)
        pe = state.item(1)
        pd = state.item(2)
        phi = state.item(6)
        theta = state.item(7)
        psi = state.item(8)

        # Get transformed faces
        F = drawPlane(pn, pe, pd, phi, theta, psi)

        # Draw each face and store handles
        for face in F:
            poly = Poly3DCollection([face], facecolors='lightblue', linewidths=1, edgecolors='black')
            self.ax.add_collection3d(poly)
            self.poly_handle.append(poly)

        plt.draw()
        plt.pause(0.001)


    def update(self,state, time):
        self.draw(state, time)