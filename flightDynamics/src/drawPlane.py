import numpy as np
from numpy import cos as c
from numpy import sin as s

    # Body to Inertial
def rotBody2Inertial(phi, theta, psi):
    R = np.array([[c(theta)*c(psi), s(phi)*s(theta)*c(psi)-c(phi)*s(psi), c(phi)*s(theta)*c(psi)+s(phi)*s(psi)],
                            [c(theta)*s(psi), s(phi)*s(theta)*s(psi)+c(phi)*c(psi), c(phi)*s(theta)*s(psi)-s(phi)*c(psi)],
                            [-s(theta), s(phi)*c(theta), c(phi)*c(theta)]]) 
    return R

def rotInertial2Body(phi,theta,psi):
    R = np.array([[c(theta)*c(psi), s(phi)*s(theta)*c(psi)-c(phi)*s(psi), c(phi)*s(theta)*c(psi)+s(phi)*s(psi)],
                            [c(theta)*s(psi), s(phi)*s(theta)*s(psi)+c(phi)*c(psi), c(phi)*s(theta)*s(psi)-s(phi)*c(psi)],
                            [-s(theta), s(phi)*c(theta), c(phi)*c(theta)]]) 
    return R.T

def rotInertialFixed(phi, theta, psi):
    R_phi = np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]])
    R_theta = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]])
    R_psi = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]])

    # Apply in inertial (fixed) frame: R = Rx(phi) @ Ry(theta) @ Rz(psi)
    return R_phi @ R_theta @ R_psi


def drawPlane(pn, pe, pd, phi, theta, psi):
    # scale = 0.8
    scale = 5

    fuse_l1 = 2.0
    fuse_l2 = 0.1
    fuse_l3 = 5
    fuse_w = 1
    fuse_h = 1
    wing_l = 1.5
    wing_w = 7
    # tail_l     = 1
    tail_h = 1.5
    tailwing_w = 3.3
    tailwing_l = 0.75

    # Drawing Points
    V = scale * np.array([
        # Front Point
        [fuse_l1, 0, 0],
        # Square joining cockpit to fuselage
        [fuse_l2, fuse_w / 2, -fuse_h / 2],
        [fuse_l2, -fuse_w / 2, -fuse_h / 2],
        [fuse_l2, -fuse_w / 2, fuse_h / 2],
        [fuse_l2, fuse_w / 2, fuse_h / 2],
        # Back tip of plane
        [-fuse_l3, 0, 0],
        # Wings
        [0, 0.5 * wing_w, 0],
        [-wing_l, 0.5 * wing_w, 0],
        [-wing_l, -0.5 * wing_w, 0],
        [0, -0.5 * wing_w, 0],
        # Horizontal Stabilizer
        [-(fuse_l3 - tailwing_l), .5 * tailwing_w, 0],
        [-fuse_l3, .5 * tailwing_w, 0],
        [-fuse_l3, -.5 * tailwing_w, 0],
        [-(fuse_l3 - tailwing_l), -.5 * tailwing_w, 0],
        # Other 2 points for vertical stabilizer (third is back of plane)
        [-(fuse_l3 - tailwing_l), 0, 0],
        [-fuse_l3, 0, -tail_h]]).T

    n = np.shape(V)[1]
    R = rotBody2Inertial(phi, theta, psi)
    T = np.array([[pn], [pe], [pd]]) * np.ones((3, n))
    V = np.matmul(R, V) + T
    R_plot = np.array([[0, 1, 0],
                    [1, 0, 0],
                    [0, 0, -1]])

    V = np.matmul(R_plot, V)

    # For Plotting Purposes to avoid messing up point naming and indexing
    v1 = 0
    v2 = 1
    v3 = 2
    v4 = 3
    v5 = 4
    v6 = 5
    v7 = 6
    v8 = 7
    v9 = 8
    v10 = 9
    v11 = 10
    v12 = 11
    v13 = 12
    v14 = 13
    v15 = 14
    v16 = 15

    F = [[V[:, v1], V[:, v2], V[:, v3], V[:, v1]],
        [V[:, v1], V[:, v3], V[:, v4], V[:, v1]],
        [V[:, v1], V[:, v2], V[:, v5], V[:, v1]],
        [V[:, v1], V[:, v4], V[:, v5], V[:, v1]],
        # Fuselage
        [V[:, v2], V[:, v3], V[:, v6], V[:, v2]],
        [V[:, v3], V[:, v4], V[:, v6], V[:, v3]],
        [V[:, v4], V[:, v5], V[:, v6], V[:, v4]],
        [V[:, v2], V[:, v5], V[:, v6], V[:, v4]],
        # Wings
        [V[:, v7], V[:, v8], V[:, v9], V[:, v10], V[:, v7]],
        # Horizontal Stabilizer
        [V[:, v11], V[:, v12], V[:, v13], V[:, v14], V[:, v11]],
        # Vertical Stabilizer
        [V[:, v6], V[:, v15], V[:, v16], V[:, v6]]]

    return F