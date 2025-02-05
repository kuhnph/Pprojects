import numpy as np
from param import param
P = param()

class controller:
    def __init__(self,kp_z,kd_z,kp_theta,kd_theta,kp_h,kd_h):
        self.kp_z = kp_z
        self.kd_z = kd_z
        self.kp_theta = kp_theta
        self.kd_theta = kd_theta
        self.kp_h = kp_h
        self.kd_h = kd_h
    
    def update(self, H_r, Z_r, state):
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        zDot = state.item(3)
        hDot = state.item(4)
        thetaDot = state.item(5)

        # Longitude control (vertical dynamics)
        F = self.kp_h * (H_r - h) - self.kd_h * hDot + P.g * P.m

        # Latitude control (horizontal dynamics)
        theta_r = self.kp_z * (Z_r - z) - self.kd_z * zDot
        T_r = self.kp_theta * (theta_r - theta) - self.kd_theta * thetaDot

        # Decouple torque into forces
        Fr = F / 2 + T_r / P.L
        Fl = F / 2 - T_r / P.L

        Fr = self.saturate(Fr)
        Fl = self.saturate(Fl)

        u = np.array([Fl, Fr])
        return u

    def saturate(self, u):
        if abs(u) > P.limit:
            u = P.limit*np.sign(u)
        return u