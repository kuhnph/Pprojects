import numpy as np

class params:
    def __init__(self):
        #simulation params
        self.Ts = 0.005
        self.T_end = 8
        self.plot_delimination = 10

        #Pysical properties
        self.noZero = 1e-3
        self.m = 1.56
        self.g = 9.81
        self.Jx = 0.1147
        self.Jy = 0.0576 
        self.Jz = 0.1712
        self.Jxz = 0.0015
        self.S = 0.2589
        self.b = 1.4224
        self.c = 0.3302
        self.S_prop = 0.0314
        self.rho = 1.2683
        self.k_motor = 20
        self.k_Tp = 0
        self.k_Omega = 0
        self.e = 0.9
        self.AR = self.b**2/self.S

        #Longitudinal
        self.CL_0 = 0.09167
        self.CD_0 = 0.01631
        self.Cm_0 = -0.02338
        self.CL_alpha = 3.5016
        self.CD_alpha = 0.2108
        self.Cm_alpha = -0.5675
        self.Cl_q = 2.8932
        self.CD_q = 0
        self.Cm_q = -1.3990
        self.CL_del_e = 0.2724
        self.CD_del_e = 0.3045
        self.CM_del_e = -0.3254
        self.C_prop = 1.0
        self.M = 50
        self.alpha0 = 0.4712
        self.epsilon = 0.1592
        self.CD_p = 0.0254
        self.Cn_del_r = -0.00328

        #Latera
        self.CY_0 = 0
        self.Cl_0 = 0
        self.Cn_0 = 0
        self.CY_beta = -0.07359
        self.Cl_beta = -0.02854
        self.Cn_beta = -0.00040
        self.CY_p = 0
        self.Cl_p = -0.3209
        self.Cn_p = -0.01297
        self.CY_r = 0
        self.Cl_r = 0.03066
        self.Cn_r = -0.00434
        self.CY_del_alpha = 0
        self.Cl_del_alpha = 0.1682
        self.Cn_del_alpha = -0.00328
        self.CY_del_r = -0.17
        self.Cl_del_r = 0.105

        #Torque things
        self.T = self.Jx*self.Jz - self.Jxz**2
        self.T1 = (self.Jxz*(self.Jx-self.Jy+self.Jz))/self.T
        self.T2 = (self.Jz*(self.Jz-self.Jy) + self.Jxz**2)/self.T
        self.T3 = self.Jz/self.T
        self.T4 = self.Jxz/self.T
        self.T5 = (self.Jz-self.Jx)/self.Jy
        self.T6 = self.Jxz/self.Jy
        self.T7 = ((self.Jx-self.Jy)*self.Jx + self.Jxz**2)/self.T
        self.T8 = self.Jx/self.T



        self.state0 = np.array([[0], #Pn
                           [0], #Pe
                           [0], #Pd
                           [5], #u
                           [0], #v
                           [0], #w
                           [np.radians(0)], #phi
                           [np.radians(-90)], #theta
                           [np.radians(0)], #psi
                           [0], #p
                           [0], #q
                           [0]])#r

    def R_body_to_inertial(self, phi, theta, psi):
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)

        R = np.array([
            [c_theta * c_psi, s_phi * s_theta * c_psi - c_phi * s_psi, c_phi * s_theta * c_psi + s_phi * s_psi],
            [c_theta * s_psi, s_phi * s_theta * s_psi + c_phi * c_psi, c_phi * s_theta * s_psi - s_phi * c_psi],
            [-s_theta,        s_phi * c_theta,                        c_phi * c_theta]
        ])
        return R