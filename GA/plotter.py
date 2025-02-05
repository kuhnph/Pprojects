import matplotlib.pyplot as plt
import numpy as np

class livePlot:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'ro-', label="Live Data")
        self.x_data = []
        self.y_data = []
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Value")
        self.ax.legend()
        plt.ion()
        plt.show()

    def update_plot(self, array):
        # Extract x and y values
        time_val = array[-1]   # Last value is the x-axis (time)
        y_val = array[1]       # Second value is the y-axis

        # Update data
        self.x_data.append(time_val)
        self.y_data.append(y_val)

        # Update plot
        self.line.set_xdata(self.x_data)
        self.line.set_ydata(self.y_data)

        # Rescale axes
        self.ax.relim()
        self.ax.autoscale_view()

        # Redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

class staticPlot:
    def __init__(self,height=4,width=12):
        self.fig, self.ax = plt.subplots(1,3)
        self.fig.set_size_inches(width,height)

    def makePlot(self, stateHistory):
        # Extract state variables from stateHistory
        stateHistory = stateHistory.T
        z = stateHistory[0]
        h = stateHistory[1]
        theta = np.degrees(stateHistory[2])
        zDot = stateHistory[3]
        hDot = stateHistory[4]
        thetaDot = stateHistory[5]
        H_r = stateHistory[6]
        Z_r = stateHistory[7]
        time = stateHistory[-1]

        # Plot height (h) vs time
        self.ax[0].plot(time, h, label='Height (h)', color='blue', linestyle='-', linewidth=1.5)
        self.ax[0].plot(time, H_r, color='purple', linestyle='-', linewidth=2.0)

        self.ax[0].set_title('Height vs Time', fontsize=12)
        self.ax[0].set_xlabel('Time (s)', fontsize=10)
        self.ax[0].set_ylabel('Height (h)', fontsize=10)
        self.ax[0].grid(True, linestyle='--', alpha=0.7)
        self.ax[0].legend()

        # Plot position (z) vs time
        self.ax[1].plot(time, z, label='Position (z)', color='green', linestyle='--', linewidth=1.5)
        self.ax[1].plot(time, Z_r, color='purple', linestyle='-', linewidth=2.0)

        self.ax[1].set_title('Position vs Time', fontsize=12)
        self.ax[1].set_xlabel('Time (s)', fontsize=10)
        self.ax[1].set_ylabel('Position (z)', fontsize=10)
        self.ax[1].grid(True, linestyle='--', alpha=0.7)
        self.ax[1].legend()

        # Plot angle (theta) vs time
        self.ax[2].plot(time, theta, label='Angle (θ)', color='red', linestyle='-.', linewidth=1.5)
        self.ax[2].set_title('Angle vs Time', fontsize=12)
        self.ax[2].set_xlabel('Time (s)', fontsize=10)
        self.ax[2].set_ylabel('Angle (θ)', fontsize=10)
        self.ax[2].grid(True, linestyle='--', alpha=0.7)
        self.ax[2].legend()

        # Adjust layout for better visibility
        plt.tight_layout()
        plt.pause(200)
        