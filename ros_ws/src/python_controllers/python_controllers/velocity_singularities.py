import numpy as np

def Rx(theta):
    return np.array([[1,0,0],
                     [0,np.cos(theta),-np.sin(theta)],
                     [0,np.sin(theta), np.cos(theta)]])

def Ry(theta):
    return np.array([[ np.cos(theta),0,np.sin(theta)],
                     [0,1,0],
                     [-np.sin(theta),0,np.cos(theta)]])

def Rz(theta):
    return np.array([[np.cos(theta),-np.sin(theta),0],
                     [np.sin(theta), np.cos(theta),0],
                     [0,0,1]])

def homogeneous(R, p):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = p
    return T

def forward_kinematics(q):
    q1,q2,q3,q4,q5 = q
    T_0b = homogeneous(Rz(np.pi), np.array([0,0,0]))
    T_b1 = homogeneous(Rz(q1), np.array([0,-0.0452,0.0165]))
    T_12 = homogeneous(Rz(q2) @ Ry(-np.pi/2), np.array([0,-0.0306,0.1025]))
    T_23 = homogeneous(Rz(q3), np.array([0.11257,-0.028,0]))
    T_34 = homogeneous(Rz(q4) @ Rz(np.pi/2), np.array([0.0052,-0.1349,0]))
    T_45 = homogeneous(Rz(q5) @ Ry(-np.pi/2), np.array([-0.0601,0,0]))
    T_5E = homogeneous(np.eye(3), np.array([0,0,0.075]))

    T_01 = T_0b @ T_b1
    T_02 = T_01 @ T_12
    T_03 = T_02 @ T_23
    T_04 = T_03 @ T_34
    T_05 = T_04 @ T_45
    T_0E = T_05 @ T_5E

    T_list = [T_01, T_02, T_03, T_04, T_05]
    return T_0E, T_list

def compute_jacobian(q):
    T_0E, T_list = forward_kinematics(q)
    p_E = T_0E[:3,3]

    Jv, Jw = [], []
    T_frames = [np.eye(4)] + T_list
    for i in range(5):
        T = T_frames[i]
        z_i = T[:3,2]
        p_i = T[:3,3]
        Jv.append(np.cross(z_i, p_E - p_i))
        Jw.append(z_i)

    Jv = np.array(Jv).T
    Jw = np.array(Jw).T
    return np.vstack((Jv,Jw)), T_0E

def inverse_term(q):
    J, _ = compute_jacobian(q) 
    return np.linalg.inv(J @ J.T)

def main(args=None):
    positions = [[-0.9203558783452418, 0.6449022068103117, -0.8764841167068065, 0.23238188827083028, 1.5707963267948966],
                [-1.3033600504244371, -0.18201407566831967, 0.4462983961525787, 1.3065120020869099, 1.5707963267948966],
                [-0.0008460179385284725, 0.9424322543473822, -0.18347354351912218, 1.5968116865659565, 1.5707963267948966],
                [-0.0005659359242338401, 0.22671913802064328, 0.8207868286320217, 1.3082643308428328, 1.5707963267948966]
    ]

    for position in positions:
        print(inverse_term(position))

if __name__ == "__main__":
    main()