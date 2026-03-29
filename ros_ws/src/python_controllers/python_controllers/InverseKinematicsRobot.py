from scipy.optimize import minimize, OptimizeResult
import numpy as np
from numpy.typing import NDArray

np.set_printoptions(precision=4, suppress=True)

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

AngleVector      = NDArray[np.float64]  # shape (3,)  — Euler angles [rx, ry, rz]
JointAngles      = NDArray[np.float64]  # shape (5,)  — [shoulder, lower, upper, wrist, gripper]
TransformMatrix  = NDArray[np.float64]  # shape (4, 4) — homogeneous transform
EulerSolution    = tuple[AngleVector, AngleVector]  # two Euler-angle solutions
IKSolution       = tuple[JointAngles | str, float]  # (angles_or_"No Solution", error)

# ---------------------------------------------------------------------------
# Initial-guess pool
# ---------------------------------------------------------------------------

_MAX_GUESSES: int = 50

_initial_guesses: list[JointAngles] = [
    np.array([ 0.0,  0.0,  0.0,  0.0,  0.0]),
    np.array([ 0.0, -0.5,  0.5,  0.5,  0.0]),
    np.array([ 0.5, -1.0,  1.0,  1.0,  0.0]),
    np.array([-0.5, -1.0,  0.5,  1.5,  0.0]),
    np.array([ 0.0, -1.5,  1.0,  2.0,  0.0]),
    np.array([ 1.0, -0.5,  0.5,  1.0,  0.0]),
    np.array([-1.0, -0.5,  0.5,  1.0,  0.0]),
    np.array([ 0.0, -1.0, -1.0,  2.0,  0.0]),
]

# ---------------------------------------------------------------------------
# Guess-pool management
# ---------------------------------------------------------------------------

def usefull(new_angles: JointAngles) -> bool:
    """
    Add a joint configuration to the initial-guess pool if it is sufficiently
    different from all existing entries.

    The new guess is inserted at the front of the list so that the most recently
    successful configuration is tried first on the next IK call. The pool is
    capped at _MAX_GUESSES entries..
    """
    global _initial_guesses

    new_angles = np.array(new_angles)
    similarity_threshold: float = 0.05  # radians

    for existing in _initial_guesses:
        if np.linalg.norm(new_angles - existing) < similarity_threshold:
            return False

    _initial_guesses.insert(0, new_angles)

    if len(_initial_guesses) > _MAX_GUESSES:
        _initial_guesses = _initial_guesses[:_MAX_GUESSES]

    return True

# ---------------------------------------------------------------------------
# Rotation & homogeneous transformation
# ---------------------------------------------------------------------------

def rotation_matrix(angle: float, axis: int) -> NDArray[np.float64]:
    """
    Build a 3×3 rotation matrix for a single-axis rotation.
    """
    c, s = np.cos(angle), np.sin(angle)

    if axis == 0:
        return np.array([[1,  0,  0],
                         [0,  c, -s],
                         [0,  s,  c]])
    if axis == 1:
        return np.array([[ c,  0,  s],
                         [ 0,  1,  0],
                         [-s,  0,  c]])
    if axis == 2:
        return np.array([[c, -s,  0],
                         [s,  c,  0],
                         [0,  0,  1]])

    raise ValueError("axis must be 0, 1, or 2")


def Transformation(
    angle_vector: AngleVector,
    translation_vector: NDArray[np.float64] = np.zeros(3),
) -> TransformMatrix:
    """
    Build a 4×4 homogeneous transformation matrix from intrinsic ZYX Euler angles
    and an optional translation.
    """
    T = np.eye(4)
    T[:3, 3] = translation_vector
    T[:3, :3] = (
          rotation_matrix(angle_vector[2], 2)
        @ rotation_matrix(angle_vector[1], 1)
        @ rotation_matrix(angle_vector[0], 0)
    )
    return T

# ---------------------------------------------------------------------------
# Constant link transforms  (robot URDF / DH parameters)
# ---------------------------------------------------------------------------

T_WOB = Transformation(np.array([0, 0, np.pi]))
T_BS  = Transformation(np.zeros(3),          np.array([ 0.0,    -0.0452,  0.0165]))
T_SU  = Transformation(np.array([0, -np.pi/2, 0]), np.array([ 0.0,    -0.0306,  0.1025]))
T_UL  = Transformation(np.zeros(3),          np.array([ 0.11257, -0.028,   0.0   ]))
T_LW  = Transformation(np.array([0, 0, np.pi/2]), np.array([ 0.0052, -0.1349,  0.0   ]))
T_WRG = Transformation(np.array([0, -np.pi/2, 0]), np.array([-0.0601,  0.0,     0.0   ]))
T_GGC = Transformation(np.zeros(3),          np.array([ 0.0,     0.0,     0.075 ]))

# ---------------------------------------------------------------------------
# Floor-collision penalty
# ---------------------------------------------------------------------------

_JOINT_BOUNDS: list[tuple[float, float]] = [
    (-2.0,         2.3        ),   # shoulder
    (-np.pi,       np.pi / 2  ),   # lower arm
    (-np.pi / 2,   np.pi / 2  ),   # upper arm
    (-np.pi / 2,   2.6        ),   # wrist
    (-np.pi,       np.pi      ),   # gripper rotation
]
def floor_penalty(
    angles: JointAngles,
    floor_z: float = 0.001,
    weight:  float = 100.0,
) -> float:
    """
    Penalise configurations where any link origin or gripper-link sample dips
    below `floor_z`.

    Joint origins and five evenly-spaced sample points along the wrist→tip
    segment are all checked. This prevents the optimiser from finding solutions
    that collide with the ground plane.
    """
    θ_s, θ_l, θ_u, θ_w, θ_g = angles

    T_i = Transformation(np.array([0, 0, θ_s]))
    T_j = Transformation(np.array([0, 0, θ_l]))
    T_k = Transformation(np.array([0, 0, θ_u]))
    T_l = Transformation(np.array([0, 0, θ_w]))
    T_g = Transformation(np.array([0, 0, θ_g]))

    T1      = T_WOB @ T_BS  @ T_i
    T2      = T1    @ T_SU  @ T_j
    T3      = T2    @ T_UL  @ T_k
    T4      = T3    @ T_LW  @ T_l
    T_wrist = T4    @ T_WRG @ T_g
    T5      = T_wrist @ T_GGC

    penalty: float = 0.0

    # Check all joint-frame origins
    for T in [T1, T2, T3, T4, T_wrist, T5]:
        z = T[2, 3]
        if z < floor_z:
            penalty += (floor_z - z) ** 2

    # Sample along the wrist → gripper-tip segment
    p_wrist = T_wrist[:3, 3]
    p_tip   = T5[:3, 3]
    num_samples: int = 5

    for t in np.linspace(0.0, 1.0, num_samples):
        z = (1.0 - t) * p_wrist[2] + t * p_tip[2]
        if z < floor_z:
            penalty += (floor_z - z) ** 2

    return weight * penalty

# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def forward_kinematics(angles: JointAngles) -> TransformMatrix:
    """
    Compute the end-effector pose from five joint angles.
    """
    θ_s, θ_l, θ_u, θ_w, θ_g = angles

    T_i = Transformation(np.array([0, 0, θ_s]))
    T_j = Transformation(np.array([0, 0, θ_l]))
    T_k = Transformation(np.array([0, 0, θ_u]))
    T_l = Transformation(np.array([0, 0, θ_w]))
    T_g = Transformation(np.array([0, 0, θ_g]))

    return T_WOB @ T_BS @ T_i @ T_SU @ T_j @ T_UL @ T_k @ T_LW @ T_l @ T_WRG @ T_g @ T_GGC

# ---------------------------------------------------------------------------
# Inverse kinematics — position only
# ---------------------------------------------------------------------------

def ik_position(
    world_xyz: list[float] | NDArray[np.float64],
    pos_tol:   float = 1e-3,
) -> IKSolution:
    """
    Solve position-only inverse kinematics (orientation unconstrained).

    Iterates over all current initial guesses and keeps the best result.
    """
    target_pos = np.array(world_xyz)

    def cost(angles: JointAngles) -> float:
        pos_error = np.linalg.norm(forward_kinematics(angles)[:3, 3] - target_pos) ** 2
        return pos_error + floor_penalty(angles)

    best_angles: JointAngles | None = None
    best_cost:   float              = 1e10

    for initial_guess in _initial_guesses:
        result: OptimizeResult = minimize(
            cost,
            initial_guess,
            bounds=_JOINT_BOUNDS,
            method='L-BFGS-B',
            tol=1e-3,
            options={'ftol': 1e-16, 'gtol': 1e-12, 'maxiter': 50_000},
        )
        if result.fun < 1e-3:
                sol += [best_angles]
    return sol

# ---------------------------------------------------------------------------
# Euler angle extraction from rotation matrix
# ---------------------------------------------------------------------------

def extract_euler_angles(
    R: NDArray[np.float64],
    z_angle: float = 0.0,
) -> EulerSolution:
    """
    Extract two sets of ZYX Euler angles from a 3×3 rotation matrix.

    Handles the gimbal-lock singularities at y = ±π/2 using the supplied
    `z_angle` as a free parameter.
    """
    y1: float = np.arcsin(-R[2, 0])
    y2: float = np.pi - y1

    def _solve_branch(y: float) -> AngleVector:
        if y == np.pi / 2:
            z = z_angle
            x = z + np.arctan2(R[0, 1], R[0, 2])
        elif y == -np.pi / 2:
            z = z_angle
            x = -z + np.arctan2(-R[0, 1], -R[0, 2])
        else:
            cos_y = np.cos(y)
            x = np.arctan2(R[2, 1] / cos_y, R[2, 2] / cos_y)
            z = np.arctan2(R[1, 0] / cos_y, R[0, 0] / cos_y)
        return np.array([x, y, z])

    return _solve_branch(y1), _solve_branch(y2)

# ---------------------------------------------------------------------------
# Inverse kinematics — position + orientation
# ---------------------------------------------------------------------------

def ik_pose(
    target_pos:       list[float] | NDArray[np.float64],
    desired_approach: list[float] | NDArray[np.float64] | None = None,
    w_pos:            float = 100.0,
    w_rot:            float = 120.0,
    pos_tol:          float = 1e-3,
) -> IKSolution:
    """
    Solve inverse kinematics for a desired end-effector position and optional
    approach direction.
    """
    target_pos = np.array(target_pos)

    # Snapshot the guess pool so additions mid-solve don't affect this run
    guesses: list[JointAngles] = list(_initial_guesses)

    # Normalise the approach vector once
    approach_vec: NDArray[np.float64] | None = None
    if desired_approach is not None:
        approach_vec = np.array(desired_approach, dtype=float)
        approach_vec /= np.linalg.norm(approach_vec)

    def cost(angles: JointAngles) -> float:
        T = forward_kinematics(angles)

        pos_error: float = np.linalg.norm(T[:3, 3] - target_pos) ** 2

        rot_error: float = 0.0
        if approach_vec is not None:
            gripper_z_axis = T[:3, 2]
            rot_error = np.linalg.norm(gripper_z_axis - approach_vec) ** 2

        return w_pos * pos_error + w_rot * rot_error + floor_penalty(angles)

    best_angles: JointAngles | None = None
    best_cost:   float              = 1e10

    for initial_guess in guesses:
        result: OptimizeResult = minimize(
            cost,
            initial_guess,
            bounds=_JOINT_BOUNDS,
            method='L-BFGS-B',
            options={'ftol': 1e-16, 'gtol': 1e-12, 'maxiter': 50_000},
        )
        if result.fun < best_cost:
            best_cost   = result.fun
            best_angles = result.x

        # Early exit once the solution is good enough
        if best_cost < 1e-4:
            break

    if best_cost > pos_tol:
        return 'No Solution', best_cost

    usefull(best_angles)
    return best_angles, best_cost

def euler_to_approach(rx, ry, rz):
    R = rotation_matrix(rz, 2) @ rotation_matrix(ry, 1) @ rotation_matrix(rx, 0)
    return R[:, 2]  # gripper Z axis