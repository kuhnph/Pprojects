class funcAnimation:
    def __init__(self,state):
        # Create 3D plot
        self.fig = plt.figure(figsize=(8,8))
        self.ax = self.fig.add_subplot(111, projection='3d')

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
    
    def update(self, stateHistory, timeHistory):
        pass