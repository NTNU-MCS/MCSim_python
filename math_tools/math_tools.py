import numpy as np
import math

def rad2pipi(x):
    y = np.arctan2(np.sin(x),np.cos(x))
    return y
    
def Rzyx(psi):
    """
    Rzyx(psi) computes the rotation matrix, R in SO(3), using the
    zyx convention and Euler angle representation.
    """

    R = np.array([[math.cos(psi), -math.sin(psi), 0],
                  [math.sin(psi), math.cos(psi), 0],
                  [0, 0, 1]])
    return R

def yaw2quat(psi):
    """
    Return the quternions of yaw
    """
    q1 = np.cos(psi/2)
    q4 = np.sin(psi/2)
    quat = np.array((q1, 0, 0, q4))
    return quat

def quat2eul(w, x, y, z):
    """
    Returns the ZYX roll-pitch-yaw angles from a quaternion.
    """
    q = np.array((w, x, y, z))
    #if np.abs(np.linalg.norm(q) - 1) > 1e-6:
    #   raise RuntimeError('Norm of the quaternion must be equal to 1')

    eta = q[0]
    eps = q[1:]

    S = np.array([
        [0, -eps[2], eps[1]],
        [eps[2], 0, -eps[0]],
        [-eps[1], eps[0], 0]
    ])

    R = np.eye(3) + 2 * eta * S + 2 * np.linalg.matrix_power(S, 2)

    if np.abs(R[2, 0]) > 1.0:
        raise RuntimeError('Solution is singular for pitch of +- 90 degrees')

    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = -np.arcsin(R[2, 0])
    yaw = np.arctan2(R[1, 0], R[0, 0])

    return np.array([roll, pitch, yaw])
