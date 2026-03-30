import numpy as np
import pyvista as pv
np.set_printoptions(precision=4, suppress=True)

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
    R[0:3, 0:3] = ( Rotation(angle_vector[2], 2)
        @ Rotation(angle_vector[1], 1) @ Rotation(angle_vector[0], 0)
    )
    return R

# ------------------------------------------------------------
#  Fixed transforms
# ------------------------------------------------------------

T_WOB = Transformation(np.array([0,0,np.pi]))
T_BS  = Transformation(np.zeros(3), np.array([0,-0.0452,0.0165]))
T_SU  = Transformation(np.array([0,-np.pi/2,0]), np.array([0,-0.0306,0.1025]))
T_UL  = Transformation(np.zeros(3), np.array([0.11257,-0.028,0]))
T_LW  = Transformation(np.array([0,0,np.pi/2]), np.array([0.0052,-0.1349,0]))
T_WRG = Transformation(np.array([0,-np.pi/2,0]), np.array([-0.0601,0,0]))
T_GGC = Transformation(np.zeros(3), np.array([0,0,0.075]))

T_WRGC = T_WRG @ T_GGC
T_LWGC = T_LW @ T_WRGC
P_GC = np.array([0,0,0,1])
P_WR = T_WRGC @ P_GC
TWOGC = T_WOB @ T_BS @ T_SU @ T_UL @ T_LW @ T_WRG @ T_GGC

# ------------------------------------------------------------
#  Angle grids
# ------------------------------------------------------------

shoulder_angle  = np.linspace(-2.16,  2.0,  50)
lower_arm_angle = np.linspace(-1.935, 1.945,  50)
upper_arm_angle = np.linspace(-1.586,1.792, 50)
wrist_angle     = np.linspace(-1.764,1.772, 25)

# wrist_angle     = np.linspace(0,2*np.pi, 50)
# shoulder_angle  = np.linspace(0,2*np.pi,  75)
# upper_arm_angle = np.linspace(0,2*np.pi, 50)
# lower_arm_angle = np.linspace(0,2*np.pi,  50)

Ti = np.stack([Transformation(np.array([0,0,a])) for a in shoulder_angle]).astype(np.float32)
Tj = np.stack([Transformation(np.array([0,0,a])) for a in lower_arm_angle]).astype(np.float32)
Tk = np.stack([Transformation(np.array([0,0,a])) for a in upper_arm_angle]).astype(np.float32)
Tl = np.stack([Transformation(np.array([0,0,a])) for a in wrist_angle]).astype(np.float32)

# ------------------------------------------------------------
#  1-deep: l @ P_WR
# ------------------------------------------------------------
P_l    = Tl @ P_WR          # (15, 4)
Points1 = P_l.T             # (4, 15)

cloud1 = pv.PolyData(Points1[:3].T)
pl1 = pv.Plotter()
pl1.add_points(cloud1, color='blue', point_size=6, render_points_as_spheres=True)
pl1.add_axes()
pl1.show_grid()
pl1.show(title="1-deep: wrist sweep")

# ------------------------------------------------------------
#  2-deep: k @ T_LW @ (l @ P_WR)
# ------------------------------------------------------------
P_l_exp  = P_l[None, :, :]            # (1, 15, 4)
Tk_exp   = Tk[:, None, :, :]          # (15, 1, 4, 4)
P_k      = Tk_exp @ T_LW @ P_l_exp[..., None]   # (15, 15, 4, 1)
Points2  = P_k.squeeze().reshape(-1, 4).T        # (4, N)

cloud2 = pv.PolyData(Points2[:3].T)
pl2 = pv.Plotter()
pl2.add_points(cloud2, color='blue', point_size=4, render_points_as_spheres=True)
pl2.add_axes()
pl2.show_grid()
pl2.show(title="2-deep: wrist + upper-arm sweep")

# ------------------------------------------------------------
#  3-deep: j @ T_UL @ (k @ T_LW @ (l @ P_WR))
# ------------------------------------------------------------
Tj_exp = Tj[:, None, None, :, :]      # (20, 1, 1, 4, 4)
P_j    = Tj_exp @ T_UL @ P_k          # (20, 15, 15, 4, 1)
Points3 = P_j.squeeze().reshape(-1, 4).T

cloud3 = pv.PolyData(Points3[:3].T)
pl3 = pv.Plotter()
pl3.add_points(cloud3, color='blue', point_size=2, render_points_as_spheres=True)
pl3.add_axes()
pl3.show_grid()
pl3.show(title="3-deep: wrist + upper + lower-arm sweep")

# ------------------------------------------------------------
#  4-deep: i @ T_SU @ (j @ T_UL @ (k @ T_LW @ (l @ P_WR)))
# ------------------------------------------------------------
Ti_exp  = Ti[:, None, None, None, :, :]   # (20, 1, 1, 1, 4, 4)
P_i     = Ti_exp @ T_SU @ P_j             # (20, 20, 15, 15, 4, 1)
Points4 = P_i.squeeze().reshape(-1, 4).T  # (4, N)

# Final world transform
Points4 = (T_WOB @ T_BS) @ Points4

cloud4 = pv.PolyData(Points4[:3].T)
pl4 = pv.Plotter()
pl4.add_points(cloud4, color='red', point_size=2, render_points_as_spheres=True)
pl4.add_axes()
pl4.show_grid()
pl4.show(title="4-deep: full workspace")

mask = Points4[:3][0] > 0
Points4 = np.array([Points4[:3][0][mask],Points4[:3][1][mask],Points4[:3][2][mask]])

cloud4 = pv.PolyData(Points4.T)
cloud4_decimated = cloud4.voxel_downsample(voxel_size=0.01)  
hull = cloud4_decimated.delaunay_3d().extract_surface()

pl4 = pv.Plotter()
pl4.add_mesh(hull, color='red', opacity=0.6, show_edges=True, 
             edge_color='darkred', smooth_shading=True)
pl4.add_axes()
pl4.show_grid()
pl4.show(title="4-deep: full workspace")