from params import params
import numpy as np
from numpy import cos, sin

P = params()

class FaM:
    def __init__(self):
        pass

    def FaM_Calc(state, U):
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

        del_e = U.item(0)
        del_t = U.item(1)
        del_a = U.item(2)
        del_r = U.item(3)

        #Define Va, alpha, beta up here somewhere
        Va = max(np.sqrt(u**2+v**2+w**2), P.noZero)    #this doesn't account for wind
        alpha = np.arctan2(w,u)            #this also doesn't account for wind
        beta = np.arctan2(v,np.sqrt(u**2+v**2+w**2))

        #Lift Coefficient
        num = 1 + np.exp(-P.M * (alpha - P.alpha0)) + np.exp(P.M * (alpha + P.alpha0))
        den = (1 + np.exp(-P.M * (alpha - P.alpha0))) * (1 + np.exp(P.M * (alpha + P.alpha0)))
        sigma = num/den
        CL = (1-sigma)*(P.CL_0+P.CL_alpha*alpha) + sigma*(2*np.sign(alpha)*sin(alpha)**2*cos(alpha))

        # CL = (np.pi*P.AR)/(1 + np.sqrt(1+(P.AR/2)**2))
        CD = P.CD_p + (P.CL_0 + P.CL_alpha*alpha)**2/(np.pi*np.e*P.AR)

        CX = -CD*cos(alpha) + CL*sin(alpha)
        CX_q = -P.CD_q*cos(alpha) + P.Cl_q*sin(alpha)
        CX_del_e = -P.CD_del_e*cos(alpha) + P.CL_del_e*sin(alpha)
        CZ = -CD*sin(alpha) - CL*cos(alpha)
        CZ_q = -P.CD_q*sin(alpha) - P.Cl_q*cos(alpha)
        CZ_del_e = -P.CD_del_e*sin(alpha) - P.CL_del_e*cos(alpha)

        #Forces
        T1 = np.array([[-P.m*P.g*sin(theta)],
                       [P.m*P.g*cos(theta)*sin(phi)],
                       [P.m*P.g*cos(theta)*cos(phi)]])

        T2 = .5*P.rho*Va**2*P.S*np.array([[CX+CX_q*P.c/(2*Va)*q + CX_del_e*del_e],
                                          [P.CY_0+P.CY_beta*beta + P.CY_p*P.b/(2*Va)*p + P.CY_r*P.b/(2*Va)*r + P.CY_del_alpha*del_a + P.CY_del_r*del_r],
                                          [CZ + CZ_q*P.c/(2*Va)*q + CZ_del_e*del_e]])

        T3 = .5*P.rho*P.S_prop*P.C_prop*np.array([[(P.k_motor*del_t)**2-Va**2],
                                                  [0],
                                                  [0]])

        F = T1+T2+T3

        M = .5*P.rho*Va**2*P.S*np.array([[P.b*(P.Cl_0+P.Cl_beta*beta+P.Cl_p*P.b/(2*Va)*p + P.Cl_r*P.b/(2*Va)*r + P.Cl_del_alpha*del_a + P.Cl_del_r*del_r) + -P.k_Tp*(P.k_Omega*del_t)**2],
                                         [P.c*( P.Cm_0+ P.Cm_alpha*alpha + P.Cm_q*P.c/(2*Va)*q + P.CM_del_e*del_e)],
                                         [P.b*(P.Cn_0 + P.Cn_beta*beta + P.Cn_p*P.b/(2*Va)*p + P.Cn_r*P.b/(2*Va)*r + P.Cn_del_alpha*del_a + P.Cn_del_r*del_r)]])
        
        return F, M