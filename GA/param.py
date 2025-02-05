import numpy as np
from numpy import cos,sin,array

class param:
    def __init__(self):
        self.L = .5
        self.mc = 1
        self.mf = .3
        self.m = self.mc+2*self.mf
        self.b = 0.0
        # self.j = 0.0042
        self.j = self.L**2*self.mf
        self.g = 9.81
        self.limit = self.g * self.m
        self.simTimeStep = 0.1
        self.animationTimeStep = 0.01
        self.boxWidth = 0.1
        self.m = self.mc+2*self.mf
        self.F_max = 20
        self.t_end = 10

        #intial States
        self.z0 = 0
        self.h0 = 0
        self.theta0 = np.radians(0)
        self.zDot0 = 0
        self.hDot0 = 0
        self.thetaDot0 = np.radians(0)

        #gain calculation
        tr_h = 2
        omega_H = 2.2/tr_h
        zeta_H = .707
        # self.kp_H = omega_H**2*self.m
        # self.kd_H = self.m*(2*zeta_H*omega_H) - self.b
        self.kp_H = omega_H**2*self.m + self.b/self.m
        self.kd_H = self.m*(2*zeta_H*omega_H)


        tr_theta=0.8
        omega_theta = 2.2/tr_theta
        zeta_theta = 0.707
        # self.Kp_theta = 2*self.j*omega_theta**2
        # self.Kd_theta = 4*self.j*zeta_theta*omega_theta
        self.kp_theta = 2*self.L**2*self.mf*omega_theta**2
        self.kd_theta = 4*self.L**2*self.mf*omega_theta*zeta_theta-self.b

        K_DC = 1
        tr_z = 10*tr_theta
        omega_z = 2.2/tr_z
        zeta_z = .707
        # self.Kp_z = omega_z/(K_DC*self.g)
        # self.Kd_z = (2*zeta_z*omega_z+self.b/self.m)/(self.g*K_DC)
        self.kp_z = (-2*omega_z*zeta_z) / (self.g*(self.mc-2*self.mf))
        self.kd_z = (self.b-self.mc*omega_z**2-2*self.mf*omega_z**2) / (self.g*(self.mc+2*self.mf))

        #Hardcode gains
        self.kp_z,self.kd_z,self.kp_theta,self.kd_theta = [-0.10915795618141057, -0.18056677658776898, -2.6752663546611135, -1.321137615263797]
        self.kp_h, self.kd_h = [2.698529209947056, 3.742713474710464]
        self.U_H=6
        self.U_Z=4

    
    def R(self, pointsX,pointsY, state):
        z = state.item(0)
        h = state.item(1)
        angle = state.item(2)
        rot = array([[cos(angle),-sin(angle)],
                    [sin(angle), cos(angle)]])
        
        pointsX = pointsX-np.ones(len(pointsX))*z
        pointsY = pointsY-np.ones(len(pointsY))*h
        P = list(zip(pointsX,pointsY))

        pointsX, pointsY = zip(*[tuple(np.matmul(rot,i)) for i in P])
        pointsX = pointsX+np.ones(len(pointsX))*z
        pointsY = pointsY+np.ones(len(pointsY))*h

        return pointsX, pointsY
