#!/usr/bin/env python3
import numpy as np

# UR5e Robot Configuration
SENSOR_NAMES = [
    "shoulder_pan_joint_sensor", 
    "shoulder_lift_joint_sensor",
    "elbow_joint_sensor", 
    "wrist_1_joint_sensor",
    "wrist_2_joint_sensor", 
    "wrist_3_joint_sensor"
]

MOTOR_NAMES = [
    "shoulder_pan_joint", 
    "shoulder_lift_joint",
    "elbow_joint", 
    "wrist_1_joint",
    "wrist_2_joint", 
    "wrist_3_joint"
]

# UR5e geometric model information
iTj_0 = [
    np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
    np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0.1625], [0, 0, 0, 1]]),
    np.array([[0, 1, 0, 0], [-1, 0, 0, 0.425], [0, 0, 1, 0], [0, 0, 0, 1]]),
    np.array([[0, -1, 0, -0.3922], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]),
    np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0.1333], [0, 0, 0, 1]]),
    np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0.0997], [0, 0, 0, 1]]),
]

# The number of joints here should be equal to the number of matrices in iTj_0
JOINT_TYPE = [0, 0, 0, 0, 0, 0]  # 0 for revolute, 1 for prismatic

# Tool transformation relative to the end-effector frame
eTt = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0.0996],
    [0, 0, 0, 1]
])

# Initial joint configuration
Q0 = [0, 0, 0, 0, 0, 0]

# Control parameters
K_ANGULAR = 0.4
K_LINEAR = 0.4
USE_TOOL = True

# Simulation parameters
SAMPLES = 1000
T_START = 0.0
T_END = 20.0

# Joint limits
QMIN = -2 * np.pi * np.ones(6)
QMAX = 2 * np.pi * np.ones(6)
MAX_VEL = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

# Convergence thresholds
POS_THRESHOLD = 0.01
ORI_THRESHOLD = 0.05