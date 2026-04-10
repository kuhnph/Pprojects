# src/sim/FaM.py
from __future__ import annotations

import numpy as np
from numpy import cos, sin
from sim.params import params


class FaM:
    def __init__(self, P: params | None = None):
        self.P = P if P is not None else params()

    def FaM_Calc(self, state, U):
        P = self.P

        # States
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        phi = state.item(6)
        theta = state.item(7)
        p = state.item(9)
        q = state.item(10)
        r = state.item(11)

        # Inputs: [delta_e, delta_t, delta_a, delta_r]
        delta_e = U.item(0)
        delta_t = U.item(1)
        delta_a = U.item(2)
        delta_r = U.item(3)

        # Air-data quantities
        Va = max(np.sqrt(u**2 + v**2 + w**2), P.noZero)
        alpha = np.arctan2(w, u)
        beta = np.arcsin(np.clip(v / Va, -1.0, 1.0))

        # Lift coefficient blending function
        num = 1.0 + np.exp(-P.M * (alpha - P.alpha0)) + np.exp(P.M * (alpha + P.alpha0))
        den = (1.0 + np.exp(-P.M * (alpha - P.alpha0))) * (1.0 + np.exp(P.M * (alpha + P.alpha0)))
        sigma = num / den

        C_L = (1.0 - sigma) * (P.C_L_0 + P.C_L_alpha * alpha) + sigma * (
            2.0 * np.sign(alpha) * sin(alpha) ** 2 * cos(alpha)
        )

        C_D = P.C_D_p + (P.C_L_0 + P.C_L_alpha * alpha) ** 2 / (np.pi * P.e * P.AR)

        # Body-axis aerodynamic coefficients
        C_X = -C_D * cos(alpha) + C_L * sin(alpha)
        C_X_q = -P.C_D_q * cos(alpha) + P.C_L_q * sin(alpha)
        C_X_delta_e = -P.C_D_delta_e * cos(alpha) + P.C_L_delta_e * sin(alpha)

        C_Z = -C_D * sin(alpha) - C_L * cos(alpha)
        C_Z_q = -P.C_D_q * sin(alpha) - P.C_L_q * cos(alpha)
        C_Z_delta_e = -P.C_D_delta_e * sin(alpha) - P.C_L_delta_e * cos(alpha)

        # Gravity force resolved in body frame
        F_gravity = np.array([
            [-P.m * P.g * np.sin(theta)],
            [ P.m * P.g * np.cos(theta) * np.sin(phi)],
            [ P.m * P.g * np.cos(theta) * np.cos(phi)],
        ], dtype=np.float64)

        # Aerodynamic forces
        F_aero = 0.5 * P.rho * Va**2 * P.S_wing * np.array([
            [C_X + C_X_q * (P.c / (2.0 * Va)) * q + C_X_delta_e * delta_e],
            [P.C_Y_0 + P.C_Y_beta * beta + P.C_Y_p * (P.b / (2.0 * Va)) * p
             + P.C_Y_r * (P.b / (2.0 * Va)) * r + P.C_Y_delta_a * delta_a + P.C_Y_delta_r * delta_r],
            [C_Z + C_Z_q * (P.c / (2.0 * Va)) * q + C_Z_delta_e * delta_e],
        ], dtype=np.float64)

        # Propulsion force
        F_prop = 0.5 * P.rho * P.S_prop * P.C_prop * np.array([
            [(P.k_motor * delta_t) ** 2 - Va**2],
            [0.0],
            [0.0],
        ], dtype=np.float64)

        F = F_gravity + F_aero + F_prop

        # Moments
        M = 0.5 * P.rho * Va**2 * P.S_wing * np.array([
            [P.b * (
                P.C_ell_0
                + P.C_ell_beta * beta
                + P.C_ell_p * (P.b / (2.0 * Va)) * p
                + P.C_ell_r * (P.b / (2.0 * Va)) * r
                + P.C_ell_delta_a * delta_a
                + P.C_ell_delta_r * delta_r
            ) - P.k_T_p * (P.k_Omega * delta_t) ** 2],
            [P.c * (
                P.C_m_0
                + P.C_m_alpha * alpha
                + P.C_m_q * (P.c / (2.0 * Va)) * q
                + P.C_m_delta_e * delta_e
            )],
            [P.b * (
                P.C_n_0
                + P.C_n_beta * beta
                + P.C_n_p * (P.b / (2.0 * Va)) * p
                + P.C_n_r * (P.b / (2.0 * Va)) * r
                + P.C_n_delta_a * delta_a
                + P.C_n_delta_r * delta_r
            )],
        ], dtype=np.float64)

        return F, M