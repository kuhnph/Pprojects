# src/sim/params.py
import numpy as np
from sim.rotations import R_body_to_inertial


class params:
    def __init__(self):
        # Simulation parameters
        self.speed_scale = 8.0
        self.sim_hz = 200
        self.render_hz = 60
        self.t = 0.0
        self.Ts = 1.0 / self.sim_hz
        self.t_end = 1e5
        self.N = int(self.t_end / self.Ts)
        self.Logging = False

        # Physical properties
        self.noZero = 1e-3
        self.m = 1.56
        self.g = 9.81
        self.Jx = 0.1147
        self.Jy = 0.0576
        self.Jz = 0.1712
        self.Jxz = 0.0015

        self.S_wing = 0.2589
        self.b = 1.4224
        self.c = 0.3302
        self.S_prop = 0.0314

        self.rho = 1.2683
        self.k_motor = 20.0
        self.k_T_p = 0.0
        self.k_Omega = 0.0

        self.e = 0.9
        self.AR = self.b**2 / self.S_wing

        # Longitudinal aerodynamic coefficients
        self.C_L_0 = 0.09167
        self.C_D_0 = 0.01631
        self.C_m_0 = -0.02338

        self.C_L_alpha = 3.5016
        self.C_D_alpha = 0.2108
        self.C_m_alpha = -0.5675

        self.C_L_q = 2.8932
        self.C_D_q = 0.0
        self.C_m_q = -1.3990

        self.C_L_delta_e = 0.2724
        self.C_D_delta_e = 0.3045
        self.C_m_delta_e = -0.3254

        self.C_prop = 1.0
        self.M = 50.0
        self.alpha0 = 0.4712
        self.epsilon = 0.1592
        self.C_D_p = 0.0254

        # Lateral-directional aerodynamic coefficients
        self.C_Y_0 = 0.0
        self.C_ell_0 = 0.0
        self.C_n_0 = 0.0

        self.C_Y_beta = -0.07359
        self.C_ell_beta = -0.02854
        self.C_n_beta = 0.00040

        self.C_Y_p = 0.0
        self.C_ell_p = -0.3209
        self.C_n_p = -0.01297

        self.C_Y_r = 0.0
        self.C_ell_r = 0.03066
        self.C_n_r = -0.00434

        # Control derivatives
        self.C_Y_delta_a = 0.0
        self.C_ell_delta_a = 0.1682
        self.C_n_delta_a = -0.00328

        self.C_Y_delta_r = -0.17
        self.C_ell_delta_r = 0.105
        self.C_n_delta_r = -0.00328

        # Gamma parameters from Beard/McLain
        self.Gamma = self.Jx * self.Jz - self.Jxz**2
        self.Gamma1 = self.Jxz * (self.Jx - self.Jy + self.Jz) / self.Gamma
        self.Gamma2 = (self.Jz * (self.Jz - self.Jy) + self.Jxz**2) / self.Gamma
        self.Gamma3 = self.Jz / self.Gamma
        self.Gamma4 = self.Jxz / self.Gamma
        self.Gamma5 = (self.Jz - self.Jx) / self.Jy
        self.Gamma6 = self.Jxz / self.Jy
        self.Gamma7 = ((self.Jx - self.Jy) * self.Jx + self.Jxz**2) / self.Gamma
        self.Gamma8 = self.Jx / self.Gamma

        # Initial state: [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]^T
        self.state0 = np.array([
            [
                0.0,
                0.0,
                0.0,
                19.97213816555619,
                -3.4106051316484816e-08,
                1.055318480807477,
                1.7053025658242407e-10,
                0.152790440292938,
                0.0,
                -3.38214786100923e-10,
                3.745413743113032e-19,
                2.1963338460718988e-09,
            ]
        ], dtype=np.float64).T

        # Trim input: [delta_e, delta_t, delta_a, delta_r]^T
        self.u_star = np.array([
            [
                -0.1639169479601792,
                0.9995752159282119,
                -1.1218706735438058e-09,
                1.2740502677867338e-09,
            ]
        ], dtype=np.float64).T

        self.u = self.u_star.copy()

    def R_body_to_inertial(self, phi, theta, psi):
        return R_body_to_inertial(phi, theta, psi, dtype=np.float64)