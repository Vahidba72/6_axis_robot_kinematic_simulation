#!/usr/bin/env python3
import numpy as np

def YPRToRot(psi, theta, phi):
    """
    The function computes the rotation matrix using the YPR (Yaw-Pitch-Roll)
    convention, given psi, theta, phi.
    
    Inputs:
    - psi: angle around z axis (Yaw)
    - theta: angle around y axis (pitch)
    - phi: angle around x axis (roll)
    
    Outputs:
    - R: rotation matrix
    """      
    if abs(theta - np.pi/2) < 1e-6 or abs(theta + np.pi/2) < 1e-6:
        print('Warning: Singular configuration detected: theta = +/- pi/2. Rotation matrix may lose one degree of freedom')
    
    R_z = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])
    
    R_y = np.array([
        [np.cos(theta), 0, np.sin(theta)],  
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])
    
    # combine the rotation matrices
    R = R_z @ R_y @ R_x  
    return R   

def KinematicSimulation(q, q_dot, dt, qmin, qmax, max_vel):
    """
    Simulates the kinematic update of joint positions.
    
    Inputs:
    - q: Current joint positions (list or array)
    - q_dot: Joint velocities (list or array)
    - dt: Time step for the simulation
    - qmin: Minimum joint limits (array)
    - qmax: Maximum joint limits (array)
    - max_vel: Maximum velocity limits (array)
    
    Outputs:
    - q_dot: Updated joint velocities after applying limits
    """
    # Enforce joint limits and velocity limits
    q_next = np.array(q) + dt * q_dot.flatten()
    
    for i in range(len(q_next)):
        # Clamp velocity if joint is at/over limit
        if (q[i] <= qmin[i] and q_dot[i] < 0) or (q[i] >= qmax[i] and q_dot[i] > 0):
            q_dot[i] = 0.0
        # Clamp velocity to max allowed
        if abs(q_dot[i]) > max_vel[i]:
            q_dot[i] = np.sign(q_dot[i]) * max_vel[i]
        # Prevent overshooting the limit in one step
        if q_next[i] < qmin[i]:
            q_dot[i] = (qmin[i] - q[i]) / dt
        elif q_next[i] > qmax[i]:
            q_dot[i] = (qmax[i] - q[i]) / dt
    
    return q_dot

def parse_expr(expr):
    """
    Safe evaluation for pi expressions.
    """
    return float(eval(expr, {"__builtins__": None}, {"pi": np.pi, "np": np}))