import numpy as np


SHOULDER_LEN = 0.55
ELBOW_LEN = 0.4

def compute_joint_corrections(theta1, theta2, dx, dy):
    l1 = SHOULDER_LEN
    l2 = ELBOW_LEN
    
    J = np.array([
        [-l1 * np.sin(theta1) - l2 * np.sin(theta1 + theta2), -l2 * np.sin(theta1 + theta2)],
        [l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2), l2 * np.cos(theta1 + theta2)],
    ])
    
    J_inv = np.linalg.pinv(J)
    
    dpos = np.array([dx, dy])
    dthetas = J_inv @ dpos
    
    return dthetas