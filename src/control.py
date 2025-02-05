import numpy as np
from param import param
P = param()

class controller:
    def __init__(self,type="PID"):
        self.type = type
    
    def update(self, Zr, Hr, state):
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        zDot = state.item(3)
        hDot = state.item(4)
        thetaDot = state.item(5)

        if self.type=="PID":
            # Longitude control (vertical dynamics)
            F_r = P.kp_H * (Hr - h) - P.kd_H * hDot + P.g * P.m
            

            # Latitude control (horizontal dynamics)
            Thetar = P.kp_z * (Zr - z) - P.kd_z * zDot
            T_r = P.kp_theta * (Thetar - theta) - P.kd_theta * thetaDot
        
        if self.type == "FSFB":
            u = -P.K@state
            F_r = u.item(0) + P.g * P.m
            T_r = u.item(1)


        # Decouple torque into forces
        Fr = F_r / 2 + T_r / P.L
        Fl = F_r / 2 - T_r / P.L

        # Enforce actuator limits
        Fr = np.clip(Fr, 0, P.F_max)
        Fl = np.clip(Fl, 0, P.F_max)

        u = np.array([Fl, Fr])
        return u
