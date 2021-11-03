import numpy as np
import math

import robosuite.utils.transform_utils as T

def skew_sym(x):
    """ convert 3D vector to skew-symmetric matrix form """
    x1, x2, x3 = x.ravel()
    return np.array([
        [0, -x3, x2],
        [x3, 0, -x1],
        [-x2, x1, 0]
    ])

def rand_quat():
    """ uniform sampling of unit quaternion """
    u1 = np.random.uniform(0, 1)
    u2 = np.random.uniform(0, 1)
    u3 = np.random.uniform(0, 1)
    q1 = np.sqrt(1-u1) * np.sin(2*np.pi*u2)
    q2 = np.sqrt(1-u1) * np.cos(2*np.pi*u2)
    q3 = np.sqrt(u1) * np.sin(2*np.pi*u3)
    q4 = np.sqrt(u1) * np.cos(2*np.pi*u3)
    return np.array((q1, q2, q3, q4))

q = rand_quat()

R = T.quat2mat(q)
p = np.array([1, 2, 3])

def pose2mat(R, p):
    """ convert pose to transformation matrix """
    p0 = p.ravel()
    H = np.block([
        [R, p0[:, np.newaxis]],
        [np.zeros(3), 1]
    ])
    return H

def mat2pose(T):
    """ convert transformation matrix T to pose """
    R = T[:3,:3]
    p = T[:3,3]
    return (R, p)

def adjoint(T):
    """ adjoint representation of transformation """
    R, p = mat2pose(T)
    pR = np.matmul(skew_sym(p), R)
    return np.block([
        [R, np.zeros((3, 3))],
        [pR, R],
    ])

def exp2rot(w, theta):
    """Matrix exponential of rotations (Rodrigues' Formula)

    Convert exponential coordinates to rotation matrix 
    """
    ss_w = skew_sym(w)
    R = np.eye(3) + np.sin(theta) * ss_w + (1-np.cos(theta)) * np.matmul(ss_w, ss_w)
    return R

def rot2exp(R):
    """Matrix logarithm of rotations
    
    Convert rotation matrix to exponential coordinates
    """
    
    if np.allclose(R, np.eye(3)):
        return (np.zeros(3), 0) # w is undefined
    if np.isclose(np.trace(R), -1):
        
        if not np.isclose(R[2][2], -1):
            w = np.array([R[0][2], R[1][2], R[2][2] + 1])
            w /= np.sqrt(2 * (1 + R[2][2]))
            return (w, np.pi)
        elif not np.isclose(R[1][1], -1):
            w = np.array([R[0][1], R[1][1] + 1, R[2][1]])
            w /= np.sqrt(2 * (1 + R[1][1]))
            return (w, np.pi)
        else:
            w = np.array([R[0][0] + 1, R[1][0], R[2][0]])
            w /= np.sqrt(2 * (1 + R[0][0]))
            return (w, np.pi)
    
    theta = np.arccos(0.5 * (np.trace(R) - 1))
    ss_w = (R - R.T) / (2 * np.sin(theta))
    w = np.array([ss_w[2][1], ss_w[0][2], ss_w[1][0]])
    return (w, theta)

def exp2mat(w, v, theta):
    """Matrix exponential of rigid-body motions
    
    Convert exponential coordinates to transformation matrix
    """
    
    w_norm = np.linalg.norm(w)
    v_norm = np.linalg.norm(v)
    
    if np.isclose(w_norm, 0):
        assert np.isclose(v_norm, 1), 'norm(v) must be 1'
        new_v = v.ravel() * theta
        return np.vstack([
            np.hstack([
                exp2rot(w, theta), new_v[:,np.newaxis]
            ]),
            np.array([[0, 0, 0, 1]]),
        ])
    
    assert np.isclose(w_norm, 1), 'norm(w) must be 1'
    
    ss_w = skew_sym(w)
    new_v = (np.eye(3)*theta + (1-np.cos(theta))*ss_w + (theta-np.sin(theta))*np.matmul(ss_w, ss_w)).dot(v)
    return np.vstack([
        np.hstack([
            exp2rot(w, theta), new_v[:,np.newaxis]
        ]),
        np.array([[0, 0, 0, 1]]),
    ])

def mat2exp(T):
    """Matrix logarithm of rigid-body motions
    
    Convert transformation matrix to exponential coordinates
    """
    
    R, p = mat2pose(T)

    if np.allclose(R, np.eye(3)):
        p_norm = np.linalg.norm(p)
        w = np.zeros(3)
        return (w, p/p_norm, p_norm)

    w, theta = rot2exp(R)
    ss_w = skew_sym(w)
    G_inv = 1/theta*np.eye(3) - 0.5*ss_w + (1/theta-0.5/np.tan(theta/2))*np.matmul(ss_w, ss_w)
    v = G_inv.dot(p)

    return (w, v, theta)

