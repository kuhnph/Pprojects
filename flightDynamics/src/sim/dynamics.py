# src/sim/dynamics - 2/15/2026
import numpy as np
from sim.params import params
from sim.FaM import FaM
from plotting.dataLogging import Logger
from numpy import cos as c
from numpy import sin as s
from numpy import tan
from sim.rotations import R_body_to_inertial

P = params()
FM = FaM()
L = Logger()

class dynamics:
    def __init__(self, P: params | None = None):
        self.P = P if P is not None else params()
        self.fam = FaM(self.P)
        self.logger = Logger(N=int(self.P.N))

        self.state = self.P.state0.copy()
        self.Ts = float(self.P.Ts)
        self.T = 0.0

    def rk4(self,u):
        F1 = self.f(self.state,u)
        F2 = self.f(self.state + self.Ts/2 * F1, u)
        F3 = self.f(self.state + self.Ts/2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state = self.state + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4)

    def f(self,state,U):
        #pull states in to variables to make things a tad easier
        pn = state.item(0)
        pe = state.item(1)
        pd = state.item(2)
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

        l = M.item(0)
        m = M.item(1)
        n = M.item(2)

        # Position kinematics (body -> inertial/NED)
        PDot = R_body_to_inertial(phi, theta, psi, dtype=np.float64) @ np.array([[u, v, w]]).T

        # Translational dynamics in body frame
        VDot = np.array([[r * v - q * w, p * w - r * u, q * u - p * v]]).T + (1 / self.P.m) * np.array([[fx, fy, fz]]).T

        # Euler angle rates
        WDot = np.array(
            [
                [1, s(phi) * tan(theta), c(phi) * tan(theta)],
                [0, c(phi), -s(phi)],
                [0, s(phi) / c(theta), c(phi) / c(theta)],
            ]
        ) @ np.array([[p, q, r]]).T

        # Rotational dynamics (p,q,r)
        WDDot = (
            np.array(
                [
                    [self.P.T1 * p * q - self.P.T2 * q * r],
                    [self.P.T5 * p * r - self.P.T6 * (p**2 - r**2)],
                    [self.P.T7 * p * q - self.P.T1 * q * r],
                ]
            )
            + np.array([[self.P.T3 * l + self.P.T4 * n, 1 / self.P.Jy * m, self.P.T4 * l + self.P.T8 * n]]).T
        )

        xDot = np.concatenate([PDot, VDot, WDot, WDDot])

        return xDot

    def update(self, u):
        # Log before update
        # self.logger.log(self.T, self.state, u)
        if self.T >= self.P.T_end - self.P.Ts:
            self.logger.export()

        self.T += self.Ts
        self.rk4(u)

#Newton's laws only hold in the inertial frame. This is important