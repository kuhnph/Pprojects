from params import params
import numpy as np
from numpy import cos, sin
from dynamics import dynamics

D = dynamics()

P = params()
def trimStates(Va_star,gamma_star,R_star,alpha_star,beta_star,phi_star):

    #position
    pn_star = P.state0.item(0)
    pe_star = P.state0.item(1)
    pd_star = P.state0.item(2)

    #velocity
    V_star = Va_star*np.array([[cos(alpha_star) * cos(beta_star)],
                               [sin(beta_star)                  ],
                               [sin(alpha_star)*cos(beta_star)  ]])
    u_star = V_star.item(0)
    v_star = V_star.item(1)
    w_star = V_star.item(2)

    #angles
    theta_star = alpha_star+gamma_star #When wind is 0
    #phi_star = phi_star
    psi_star = P.state0.item(8)

    #Angular rates
    W_star = Va_star/R_star * np.array([[-sin(theta_star)],
                                        [sin(phi_star)*cos(theta_star)],
                                        [cos(phi_star)*cos(theta_star)]])
    p_star = W_star.item(0)
    q_star = W_star.item(1)
    r_star = W_star.item(2)

    x_star = np.array([[pn_star,pe_star,pd_star,u_star,v_star,w_star,phi_star,theta_star,psi_star,p_star,q_star,r_star]])

    return x_star

def trimInput(state,Va,alpha,beta):
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

    # === Elevator delta_e ===
    numerator_e = (P.Jxz * (p**2 - r**2) + (P.Jx - P.Jz) * p * r) / (0.5 * P.rho * Va**2 * P.c * P.S)

    Cm_term = P.Cm_0 + P.Cm_alpha * alpha + P.Cm_q * (P.c / (2 * Va)) * q
    del_e = (numerator_e - Cm_term) / P.CM_del_e


    # === Throttle delta_t ===
    CX = -P.CD_p * np.cos(alpha) + (P.CL_0 + P.CL_alpha * alpha) * np.sin(alpha)
    CX_q = -P.CD_q * np.cos(alpha) + P.Cl_q * np.sin(alpha)
    CX_del_e = -P.CD_del_e * np.cos(alpha) + P.CL_del_e * np.sin(alpha)

    Fx = (-r * v + q * w + P.g * np.sin(theta))
    aero_term = 0.5 * P.rho * Va**2 * P.S * (CX + CX_q * (P.c / (2 * Va)) * q + CX_del_e * del_e)
    thrust_numerator = P.m * Fx - aero_term
    thrust_denominator = 0.5 * P.rho * P.S_prop * P.C_prop * P.k_motor**2
    del_t = np.sqrt((thrust_numerator + Va**2) / thrust_denominator)


    # === Aileron delta_a and Rudder delta_r ===
    # Lateral-directional coefficients
    Cp0 = P.Cl_0
    Cp_beta = P.Cl_beta
    Cp_p = P.Cl_p
    Cp_r = P.Cl_r
    Cp_del_a = P.Cl_del_alpha
    Cp_del_r = P.Cl_del_r

    Cr0 = P.Cn_0
    Cr_beta = P.Cn_beta
    Cr_p = P.Cn_p
    Cr_r = P.Cn_r
    Cr_del_a = P.Cn_del_alpha
    Cr_del_r = P.Cn_del_r

    # Gamma coefficients
    Gamma1 = P.T1
    Gamma2 = P.T2
    Gamma7 = P.T7

    l_rhs = (-Gamma1 * p * q + Gamma2 * q * r) / (0.5 * P.rho * Va**2 * P.S * P.b) - (Cp0 + Cp_beta * beta + Cp_p * (P.b * p) / (2 * Va) + Cp_r * (P.b * r) / (2 * Va))
    n_rhs = (-Gamma7 * p * q + Gamma1 * q * r) / (0.5 * P.rho * Va**2 * P.S * P.b) - (Cr0 + Cr_beta * beta + Cr_p * (P.b * p) / (2 * Va) + Cr_r * (P.b * r) / (2 * Va))

    # Build and solve matrix
    C_matrix = np.array([[Cp_del_a, Cp_del_r],
                         [Cr_del_a, Cr_del_r]])
    rhs_vector = np.array([l_rhs, n_rhs])

    del_a, del_r = np.linalg.solve(C_matrix, rhs_vector)

    u_star = np.array([[del_e],
                       [del_t],
                       [del_a],
                       [del_r]])
    return u_star

def J(Va_star,gamma_star,R_star,alpha_star,beta_star,phi_star):
    xDot_desired = np.zeros((12,1))
    xDot_desired[2,0] = Va_star*sin(gamma_star)
    xDot_desired[8,0] = Va_star/R_star

    state_star = trimStates(Va_star,gamma_star,R_star,alpha_star,beta_star,phi_star)
    u_star = trimInput(state_star,Va_star,alpha_star,beta_star)
    xDot = D.f(state_star,u_star)