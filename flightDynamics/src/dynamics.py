import numpy as np
from params import params
from FaM import FaM
from numpy import cos as c
from numpy import sin as s
from numpy import tan

P = params()
FM = FaM()

class dynamics:
    def __init__(self):
        self.state = P.state0
        self.Ts = P.Ts

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

        self.F,self.M = FaM.FaM_Calc(state,U)

        fx = self.F.item(0)
        fy = self.F.item(1)
        fz = self.F.item(2)

        l = self.M.item(0)
        m = self.M.item(1)
        n = self.M.item(2)

        # fx = 10
        # fy = 0
        # fz = 0
        # l = 0
        # m = 0
        # n = 0


        PDot = np.array([[c(theta)*c(psi), s(phi)*s(theta)*c(psi)-c(phi)*s(psi), c(phi)*s(theta)*c(psi)+s(phi)*s(psi)],
                         [c(theta)*s(psi), s(phi)*s(theta)*s(psi)+c(phi)*c(psi), c(phi)*s(theta)*s(psi)-s(phi)*c(psi)],
                         [-s(theta), s(phi)*c(theta), c(phi)*c(theta)]]) @ np.array([[u,v,w]]).T

        VDot = np.array([[r*v-q*w, p*w-r*u, q*u-p*v]]).T + 1/P.m*np.array([[fx,fy,fz]]).T

        WDot = np.array([[1, s(phi)*tan(theta), c(phi)*tan(theta)],
                         [0, c(phi), -s(phi)],
                         [0, s(phi)/c(theta), c(phi)/c(theta)]]) @ np.array([[p,q,r]]).T

        WDDot = np.array([[P.T1*p*q - P.T2*q*r],
                          [P.T5*p*r - P.T6*(p**2-r**2)],
                          [P.T7*p*q - P.T1*q*r]]) + np.array([[P.T3*l + P.T4*n, 1/P.Jy*m, P.T4*l+P.T8*n]]).T

        xDot = np.concatenate([
            PDot,      # [pn_dot, pe_dot, pd_dot] - inertial
            VDot,      # [u_dot, v_dot, w_dot] - body
            WDot,      # [phi_dot, theta_dot, psi_dot] - inertial
            WDDot      # [p_dot, q_dot, r_dot] - body
        ])

        return xDot

    def update(self,u):
        self.rk4(u)

#Newton's laws only hold in the inertial frame. This is important