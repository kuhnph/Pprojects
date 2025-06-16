import matplotlib.pyplot as plt
from params import params

P = params()
class plotting:
    def __init__(self):
        pass

    def staticPlotState(self, stateStorage):
        # Transpose so each row is a state, and the last row is time
        stateStorage = stateStorage.T
        time = stateStorage[-1]

        # Define titles and y-labels for each state
        state_labels = [
            ("$p_n$", "North Position [m]"),
            ("$p_e$", "East Position [m]"),
            ("$p_d$", "Down Position [m]"),
            ("$u$", "Forward Velocity [m/s]"),
            ("$v$", "Side Velocity [m/s]"),
            ("$w$", "Down Velocity [m/s]"),
            ("$\\phi$", "Roll Angle [rad]"),
            ("$\\theta$", "Pitch Angle [rad]"),
            ("$\\psi$", "Yaw Angle [rad]"),
            ("$p$", "Roll Rate [rad/s]"),
            ("$q$", "Pitch Rate [rad/s]"),
            ("$r$", "Yaw Rate [rad/s]")
        ]

        # Create subplots
        fig, ax = plt.subplots(3, 4, figsize=(16, 9))
        ax = ax.flatten()

        # Loop through each state
        for i in range(12):
            ax[i].plot(time, stateStorage[i])
            ax[i].set_title(state_labels[i][0], fontsize=12)
            ax[i].set_ylabel(state_labels[i][1], fontsize=10)
            ax[i].set_xlabel("Time [s]")
            ax[i].grid(True)

        # Adjust layout
        plt.tight_layout()
        plt.show()

    def staticPlotFaM(self, FaMStorage):
        # Transpose so each row is a state, and the last row is time
        FaMStorage = FaMStorage.T
        time = FaMStorage[-1]

        # Define titles and y-labels for each state
        state_labels = [
            ("$F_x$", "Force x direction [N]"),
            ("$F_y$", "Force y direction [N]"),
            ("$F_z$", "Force Z direction [N]"),
            ("$l$", "Moment about x-axis [N-m]"),
            ("$m$", "Moment about y axis [N-m]"),
            ("$n$", "Moment about z-axis [N-m]"),
        ]

        # Create subplots
        fig, ax = plt.subplots(2, 3, figsize=(16, 9))
        ax = ax.flatten()

        # Loop through each state
        for i in range(len(FaMStorage)-1):
            ax[i].plot(time, FaMStorage[i])
            ax[i].set_title(state_labels[i][0], fontsize=12)
            ax[i].set_ylabel(state_labels[i][1], fontsize=10)
            ax[i].set_xlabel("Time [s]")
            ax[i].grid(True)

        # Adjust layout
        plt.tight_layout()
        plt.show()