from scipy.optimize import minimize
import numpy as np
np.set_printoptions(precision=4, suppress=True)

# ------------------------------------------------------------
#  Initials
# ------------------------------------------------------------

INITIAL_GUESSES = [
    np.array([ 0.0,  0.0,  0.0,  0.0,  0.0]),
    np.array([0.0,  0.5, -0.5,  0.5, 0.0]),
    np.array([0.5,  1.0, -1.0,  1.0, 0.0]),
    np.array([-0.5, 1.0, -0.5,  1.5, 0.0]),
    np.array([0.0,  1.5, -1.0,  2.0, 0.0]),
]

# ------------------------------------------------------------
#  Rotation + Transformation
# ------------------------------------------------------------

def Rotation(angle, axis: int):
    c, s = np.cos(angle), np.sin(angle)
    if axis == 0:
        return np.array([[1, 0, 0],
                         [0, c,-s],
                         [0, s, c]])
    if axis == 1:
        return np.array([[ c, 0, s],
                         [ 0, 1, 0],
                         [-s, 0, c]])
    if axis == 2:
        return np.array([[c,-s, 0],
                         [s, c, 0],
                         [0, 0, 1]])
    raise ValueError("axis must be 0, 1, or 2")

def Transformation(angle_vector: np.array, translation_vector: np.array = np.zeros(3)):
    R = np.eye(4)
    R[0:3, 3] = translation_vector
    R[0:3, 0:3] = ( 
          Rotation(angle_vector[2], 2)
        @ Rotation(angle_vector[1], 1) 
        @ Rotation(angle_vector[0], 0)
    )
    return R

T_WOB = Transformation(np.array([0,0,np.pi]))
T_BS  = Transformation(np.zeros(3), np.array([0,-0.0452,0.0165]))
T_SU  = Transformation(np.array([0,-np.pi/2,0]), np.array([0,-0.0306,0.1025]))
T_UL  = Transformation(np.zeros(3), np.array([0.11257,-0.028,0]))
T_LW  = Transformation(np.array([0,0,np.pi/2]), np.array([0.0052,-0.1349,0]))
T_WRG = Transformation(np.array([0,-np.pi/2,0]), np.array([-0.0601,0,0]))
T_GGC = Transformation(np.zeros(3), np.array([0,0,0.075]))
TWOGC = T_WOB @ T_BS @ T_SU @ T_UL @ T_LW @ T_WRG @ T_GGC

# ------------------------------------------------------------
#  Forward Kinematics
# ------------------------------------------------------------

def ForwardKinematics(angles):
    θ_s, θ_l, θ_u, θ_w, θ_g = angles
    Ti = Transformation(np.array([0, 0, θ_s]))
    Tj = Transformation(np.array([0, 0, θ_l]))
    Tk = Transformation(np.array([0, 0, θ_u]))
    Tl = Transformation(np.array([0, 0, θ_w]))
    Tg = Transformation(np.array([0, 0, θ_g]))

    return T_WOB @ T_BS @ Ti @ T_SU @ Tj @ T_UL @ Tk @ T_LW @ Tl @ T_WRG @ Tg @ T_GGC

# ------------------------------------------------------------
#  Inverse Kinematics
# ------------------------------------------------------------

def ik_position(world_xyz, pos_tol=1e-3):
    """Position-only IK. Returns (angles, error_m) or ('No Solution', error_m)."""
    target_pos = np.array(world_xyz)
    def cost(a):
        pos_err = np.linalg.norm(ForwardKinematics(a)[:3, 3] - target_pos)**2
        
        return pos_err
    
    best, best_cost = None, 1e10
    bounds = [
        (-1.9,  2.3 ),   # shoulder
        (-3.15, 0.8 ),   # lower arm
        (-1.59, 1.79),   # upper arm
        (-0.98, 2.60),   # wrist
        (-np.pi, np.pi), # gripper rotation
    ]
    for ig in INITIAL_GUESSES:

        res = minimize(cost, ig, bounds=bounds, method='L-BFGS-B',
                       options={'ftol': 1e-16, 'gtol': 1e-12, 'maxiter': 2000})
        
        if res.x[1] < 0:
            continue
        
        if res.fun < best_cost:
            best_cost = res.fun; best = res.x
    err = np.sqrt(best_cost)
    if err > pos_tol:
        return list(best), err
    return list(best), err

def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def ik_pose(target, w_pos=1.0, w_rot=1.0, pos_tol=1e-3):
    target_pos = np.array(target[:3])
    target_rot = np.array(target[3:])
    bounds = [
        (-1.9,  2.3 ),   # shoulder
        (-3.15, 0.8 ),   # lower arm
        (-1.59, 1.79),   # upper arm
        (-0.98, 2.60),   # wrist
        (-np.pi, np.pi), # gripper rotation — adjust to real limits
    ]

    def cost(angles):
        T = ForwardKinematics(angles)
        pos_err = np.linalg.norm(T[:3, 3] - target_pos)
        rot_err = np.linalg.norm(rotation_matrix_to_euler_angles(T[:3, :3]) - target_rot)
        return w_pos * pos_err**2 + w_rot * rot_err**2
    best, best_cost = None, 1e10
    global INITIAL_GUESSES
    if ik_position(target_pos)[0] != 'No Solution':
        INITIAL_GUESSES += [np.array(ik_position(target_pos)[0])]
    for ig in INITIAL_GUESSES[::-1]:   # ← add this loop
        res = minimize(cost, ig, bounds=bounds, method='L-BFGS-B')
        if res.fun < best_cost:
            best_cost = res.fun; best = res.x
    if best_cost > pos_tol:
        return 'No Solution', best_cost
    return best, best_cost

# print(ik_position([0.2, 0.2, 0.2]))
# print(ik_position([0.2, 0.1, 0.4]))
# print(ik_position([0.0, 0.0, 0.4]))
# print(ik_position([0.0, 0.0, 0.07]))
# print(ik_position([0.0, 0.0452, 0.45]))

# print(ik_pose([0.2, 0.2, 0.2 ,0.000, 1.570, 0.650]))
# print(ik_pose([0.2, 0.1, 0.4,0.000,  0.000, -1.570]))
# print(ik_pose([0.0, 0.0, 0.4, 0.000, -0.785,  1.570]))
# print(ik_pose([0.0, 0.0, 0.07,3.141,  0.000,  0.000]))
# print(ik_pose([0.0, 0.0452, 0.45, -0.785,  0.000,  3.141]))