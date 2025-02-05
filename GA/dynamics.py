import numpy as np
from param import param
from numpy import sin,cos
P = param()

class dynamics:
    def __init__(self):
        self.state = np.array([[P.z0,P.h0,P.theta0,P.zDot0,P.hDot0,P.thetaDot0]]).T

    def f(self,state,u):
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        zDot = state.item(3)
        hDot = state.item(4)
        thetaDot = state.item(5)
        Fl = u.item(0)
        Fr = u.item(1)

        # zDDot = (-(Fl + Fr)*sin(theta) - P.b*zDot)/(P.m)
        # hDDot = ((Fl + Fr)*cos(theta) - P.b*hDot - P.g*P.m)/(P.m)
        # thetaDDot = (Fl*P.L - Fr*P.L - P.b*thetaDot)/(2*P.L*2*P.mf)

        zDDot = -(P.b*zDot + (Fl+Fr)*sin(theta))/(P.mc + 2*P.mf)
        hDDot = (-P.b*hDot - P.g*(P.mc+2*P.mf) + (Fl+Fr)*cos(theta))/(P.mc + 2*P.mf)
        thetaDDot = (-P.b*thetaDot + P.L*(Fl - Fr))/(2*P.L**2*P.mf)

        xDot = np.array([[zDot,hDot,thetaDot,zDDot,hDDot,thetaDDot]]).T

        return xDot

    def update(self,u):
        self.rk4_step(u)
        return self.state


    def rk4_step(self,u):
        F1 = self.f(self.state,u)
        F2 = self.f(self.state + P.simTimeStep/2*F1,u)
        F3 = self.f(self.state + P.simTimeStep/2*F2,u)
        F4 = self.f(self.state + P.simTimeStep*F3,u)
        self.state = self.state + P.simTimeStep/6 * (F1+2*F2+2*F3+F4)
