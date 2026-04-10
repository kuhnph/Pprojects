# src/sim/dynamics.py
import numpy as np
from sim.params import params
from sim.FaM import FaM
from sim.rotations import R_body_to_inertial


class dynamics:
    def __init__(self, P: params | None = None):
        self.P = P if P is not None else params()
        self.fam = FaM(self.P)

        self.state = self.P.state0.copy()
        self.Ts = float(self.P.Ts)
        self.t = float(self.P.t)

    def rk4(self, u):
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2.0 * F1, u)
        F3 = self.f(self.state + self.Ts / 2.0 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state = self.state + self.Ts / 6.0 * (F1 + 2.0 * F2 + 2.0 * F3 + F4)

    def f(self, state, U):
        # States
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        phi = state.item(6)
        theta = state.item(7)
        psi = state.item(8)
        p = state.item(9)
        q = state.item(10)
        r = state.item(11)

        F, M = self.fam.FaM_Calc(state, U)

        fx = F.item(0)
        fy = F.item(1)
        fz = F.item(2)

        ell = M.item(0)
        m = M.item(1)
        n = M.item(2)

        # Position kinematics (body -> inertial / NED)
        PDot = R_body_to_inertial(phi, theta, psi, dtype=np.float64) @ np.array([[u], [v], [w]])

        # Translational dynamics in body frame
        VDot = np.array([
            [r * v - q * w],
            [p * w - r * u],
            [q * u - p * v],
        ], dtype=np.float64) + (1.0 / self.P.m) * np.array([[fx], [fy], [fz]], dtype=np.float64)

        # Euler angle rates
        WDot = np.array([
            [1.0, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
            [0.0, np.cos(phi), -np.sin(phi)],
            [0.0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)],
        ], dtype=np.float64) @ np.array([[p], [q], [r]], dtype=np.float64)

        # Rotational dynamics
        WDDot = np.array([
            [self.P.Gamma1 * p * q - self.P.Gamma2 * q * r + self.P.Gamma3 * ell + self.P.Gamma4 * n],
            [self.P.Gamma5 * p * r - self.P.Gamma6 * (p**2 - r**2) + m / self.P.Jy],
            [self.P.Gamma7 * p * q - self.P.Gamma1 * q * r + self.P.Gamma4 * ell + self.P.Gamma8 * n],
        ], dtype=np.float64)

        xDot = np.concatenate([PDot, VDot, WDot, WDDot], axis=0)
        return xDot

    def update(self, u):
        self.rk4(u)
        self.t += self.Ts