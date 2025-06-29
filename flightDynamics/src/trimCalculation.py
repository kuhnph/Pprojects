from params import params
import numpy as np
from numpy import cos, sin
from dynamics import dynamics

D = dynamics()
P = params()
def trimStates(Va_star,gamma_star,R_star,alpha_star,beta_star,phi_star):
    '''
    Function to calculate states based on velocity, climb angle, turning radius, angle of attack, sideslip, and roll
    '''

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
    AngularRates_star = Va_star/max(R_star,P.noZero) * np.array([[-sin(theta_star)],
                                        [sin(phi_star)*cos(theta_star)],
                                        [cos(phi_star)*cos(theta_star)]])
    p_star = AngularRates_star.item(0)
    q_star = AngularRates_star.item(1)
    r_star = AngularRates_star.item(2)

    x_star = np.array([[pn_star,pe_star,pd_star,u_star,v_star,w_star,phi_star,theta_star,psi_star,p_star,q_star,r_star]])

    return x_star

def trimInput(state,Va,alpha,beta):
    '''
    calculate the inputs (elevon, thrust, aleron, and rudder) based on the trim states, airspeed, angle of attack, and side slip angle
    '''

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
    num = 1 + np.exp(-P.M * (alpha - P.alpha0)) + np.exp(P.M * (alpha + P.alpha0))
    den = (1 + np.exp(-P.M * (alpha - P.alpha0))) * (1 + np.exp(P.M * (alpha + P.alpha0)))
    sigma = num/den

    CL = (1-sigma)*(P.CL_0+P.CL_alpha*alpha) + sigma*(2*np.sign(alpha)*sin(alpha)**2*cos(alpha))
    CD = P.CD_p + (P.CL_0 + P.CL_alpha*alpha)**2/(np.pi*np.e*P.AR)

    CX = -CD*cos(alpha) + CL*sin(alpha)
    CX_q = -P.CD_q*cos(alpha) + P.Cl_q*sin(alpha)
    CX_del_e = -P.CD_del_e*cos(alpha) + P.CL_del_e*sin(alpha)

    Fx = (-r*v + q*w + P.g*np.sin(theta))
    aero_term = P.rho*Va**2*P.S * (CX + CX_q*(P.c/(2*Va))*q + CX_del_e*del_e)
    thrust_numerator = 2*P.m*Fx - aero_term
    thrust_denominator = P.rho*P.S_prop*P.C_prop*P.k_motor**2
    del_t = np.sqrt((thrust_numerator)/thrust_denominator + Va**2/P.k_motor**2)

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

    l_rhs = (-Gamma1*p*q + Gamma2*q*r) / (0.5*P.rho*Va**2*P.S*P.b) - (Cp0 + Cp_beta*beta + Cp_p*(P.b*p)/(2*Va) + Cp_r*(P.b*r)/(2*Va))
    n_rhs = (-Gamma7*p*q + Gamma1*q*r) / (0.5*P.rho*Va**2*P.S*P.b) - (Cr0 + Cr_beta*beta + Cr_p*(P.b*p)/(2*Va) + Cr_r*(P.b*r)/(2*Va))

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

def J(alpha_star, beta_star, phi_star, Va_star, gamma_star, R_star):
    '''
    Cost function that calculates how far off the current inputs bring xDot compared to our desired states.
    Desired xDot:
    [NA]
    [NA]
    [Va*sin(gamma)]
    [0]
    [0]
    [0]
    [0]
    [0]
    [Va/R]
    [0]
    [0]
    [0]
    '''

    xDot_desired = np.zeros((12,1))
    xDot_desired[2,0] = Va_star*sin(gamma_star)
    xDot_desired[8,0] = Va_star/max(R_star,P.noZero)

    state_star = trimStates(Va_star,gamma_star,R_star,alpha_star,beta_star,phi_star)
    u_star = trimInput(state_star,Va_star,alpha_star,beta_star)
    xDot = D.f(state_star,u_star)

    return np.linalg.norm(xDot-xDot_desired)**2


def gradient_descent_trim(J_func, alpha0, beta0, phi0, Va, gamma, R, 
                          epsilon=1e-8, kappa=1e-5, max_iters=200, verbose=True):
    """
    Performs gradient descent on J_func(alpha, beta, phi) with respect to alpha, beta, and phi.

    Parameters:
        J_func     : function J(alpha, beta, phi, Va, gamma, R)
        alpha0     : initial alpha (rad)
        beta0      : initial beta (rad)
        phi0       : initial phi (rad)
        Va         : airspeed
        gamma      : flight path angle
        R          : turn radius
        epsilon    : finite difference step size
        kappa      : learning rate
        max_iters  : number of iterations
        verbose    : whether to print each step

    Returns:
        alpha, beta, phi (final values)
    """

    alpha = alpha0
    beta = beta0
    phi = phi0

    for k in range(max_iters):
        J0 = J_func(alpha, beta, phi, Va, gamma, R)

        DJ_Dalpha = (J_func(alpha + epsilon, beta, phi, Va, gamma, R) - J0) / epsilon
        DJ_Dbeta  = (J_func(alpha, beta + epsilon, phi, Va, gamma, R) - J0) / epsilon
        DJ_Dphi   = (J_func(alpha, beta, phi + epsilon, Va, gamma, R) - J0) / epsilon

        alpha -= kappa * DJ_Dalpha
        # alpha -= max(abs(kappa * DJ_Dalpha),np.pi)*sin(kappa * DJ_Dalpha) 
        beta  -= kappa * DJ_Dbeta
        phi   -= kappa * DJ_Dphi

        if verbose:
            print(f"Iter {k:3d}: Cost = {J0:.6f}, α = {np.degrees(alpha):.2f}°, "
                  f"β = {np.degrees(beta):.2f}°, ϕ = {np.degrees(phi):.2f}°")

    return alpha, beta, phi


def main():
    Va_star = 10
    gamma_star = 0
    R_star = 40
    alpha_star = 0
    beta_star = 0
    phi_star = 0

    x0 = np.array([alpha_star, beta_star, phi_star])  # initial guess
    alpha_star, beta_star, phi_star = gradient_descent_trim(J, alpha_star, beta_star, phi_star, Va_star, gamma_star, R_star, max_iters=100)

    state_star = trimStates(Va_star,gamma_star,R_star,alpha_star,beta_star,phi_star)
    u_star = trimInput(state_star,Va_star,alpha_star,beta_star)

    print('\nTrim Inputs:')
    print(u_star)

    print('\nTrim States')
    print(f'trimState = np.array([[{state_star.item(0)},{state_star.item(1)},{state_star.item(2)},{state_star.item(3)},{state_star.item(4)},{state_star.item(5)},{state_star.item(6)},{state_star.item(7)},{state_star.item(8)},{state_star.item(9)},{state_star.item(10)},{state_star.item(11)}]]).T')


main()