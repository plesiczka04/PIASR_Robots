import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge

# --- LATEX READABILITY SETTINGS ---
plt.rcParams.update({
    "font.size": 12,           # Matches standard LaTeX 12pt font
    "axes.titlesize": 14,      # Slightly larger for headers
    "axes.labelsize": 12,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "figure.dpi": 300})        # High-res for clear printing

# ------------------------------------------------------------
#  Rotation + Transformation
# ------------------------------------------------------------
def Rotation(angle, axis: int):
    c, s = np.cos(angle), np.sin(angle)
    if axis == 0: return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 1: return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    if axis == 2: return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    raise ValueError("axis must be 0, 1, or 2")

def Transformation(angle_vector: np.array, translation_vector: np.array = np.zeros(3)):
    R = np.eye(4)
    R[0:3, 3] = translation_vector
    R[0:3, 0:3] = (Rotation(angle_vector[2], 2) @ 
                   Rotation(angle_vector[1], 1) @ 
                   Rotation(angle_vector[0], 0))
    return R

# ------------------------------------------------------------
#  Setup 3-Deep Calculation
# ------------------------------------------------------------
T_SU = Transformation(np.array([0, -np.pi / 2, 0]), np.array([0, -0.0306, 0.1025]))
T_UL = Transformation(np.zeros(3), np.array([0.11257, -0.028, 0]))
T_LW = Transformation(np.array([0, 0, np.pi / 2]), np.array([0.0052, -0.1349, 0]))
T_WRG = Transformation(np.array([0, -np.pi / 2, 0]), np.array([-0.0601, 0, 0]))
T_GGC = Transformation(np.zeros(3), np.array([0, 0, 0.075]))
P_WR = (T_WRG @ T_GGC) @ np.array([0, 0, 0, 1])

#  Joint Angles 
# shoulder_angle = np.linspace(-2.16, 2.0, 50)
# lower_angle = np.linspace(-1.935, 1.945, 50)
# upper_angle = np.linspace(-1.586, 1.792, 50)
# wrist_angle = np.linspace(-1.764, 1.772, 25)

wrist_angle     = np.linspace(0,2*np.pi, 50)
shoulder_angle  = np.linspace(0,2*np.pi,  75)
upper_angle = np.linspace(0,2*np.pi, 50)
lower_angle = np.linspace(0,2*np.pi,  50)

Tj = np.stack([Transformation(np.array([0, 0, a])) for a in lower_angle]).astype(np.float32)
Tk = np.stack([Transformation(np.array([0, 0, a])) for a in upper_angle]).astype(np.float32)
Tl = np.stack([Transformation(np.array([0, 0, a])) for a in wrist_angle]).astype(np.float32)

# Tensor Broadcasting (The "Three Deep")
P_l = Tl @ P_WR
P_k = Tk[:, None, :, :] @ T_LW @ P_l[None, :, :, None]
P_j = Tj[:, None, None, :, :] @ T_UL @ P_k

# Extract raw points in shoulder frame
Points3 = P_j.squeeze().reshape(-1, 4).T
P_side = T_SU @ Points3

# ------------------------------------------------------------
#  HIGH-CONTRAST PLOT
# ------------------------------------------------------------
plt.style.use("dark_background")
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# --- Left: Side Profile (Raw Cloud) ---
ax1.scatter(P_side[1], P_side[2], s=0.3, color="#00E676", alpha=0.4)
ax1.set_title("Vertical Profile (X-Z)")
ax1.set_xlabel("Reach (m)")
ax1.set_ylabel("Height (m)")
ax1.set_aspect('equal')
ax1.grid(True, color="#333333", linestyle=":")

# --- Right: Top View (Geometric Wedge) ---
R = np.sqrt(P_side[0]**2 + P_side[1]**2)
R_min, R_max = np.min(R), np.max(R)
t_min, t_max = np.rad2deg(shoulder_angle.min()), np.rad2deg(shoulder_angle.max())

wedge = Wedge((0, 0), R_max, t_min, t_max, width=R_max-R_min, 
              facecolor="#00E5FF", edgecolor="#00B0FF", alpha=0.3)
ax2.add_patch(wedge)
limit = R_max * 1.2
ax2.set_xlim(-limit, limit); ax2.set_ylim(-limit, limit)
ax2.set_title("Top Sweep (X-Y)")
ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)")
ax2.set_aspect('equal')
ax2.grid(True, color="#333333", linestyle=":")

plt.tight_layout()

# --- EXPORT FOR LATEX ---
# plt.savefig("robot_workspace.pdf", format='pdf', bbox_inches='tight')
# plt.savefig("robot_workspace.png", dpi=300, bbox_inches='tight')
plt.show()