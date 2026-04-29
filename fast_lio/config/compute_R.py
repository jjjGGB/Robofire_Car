import numpy as np

def deg2rad(deg):
    return deg * np.pi / 180.0

def rot_x(roll_deg):
    r = deg2rad(roll_deg)
    c = np.cos(r)
    s = np.sin(r)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ], dtype=float)

def rot_y(pitch_deg):
    p = deg2rad(pitch_deg)
    c = np.cos(p)
    s = np.sin(p)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ], dtype=float)

def rot_z(yaw_deg):
    y = deg2rad(yaw_deg)
    c = np.cos(y)
    s = np.sin(y)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ], dtype=float)

def euler_to_rotation_matrix(roll, pitch, yaw, order="ZYX"):
    """
    默认采用常见约定:
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    Rx = rot_x(roll)
    Ry = rot_y(pitch)
    Rz = rot_z(yaw)

    if order == "ZYX":
        R = Rz @ Ry @ Rx
    else:
        raise ValueError(f"Unsupported order: {order}")

    return R

def build_transform(R, t):
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

if __name__ == "__main__":
    # radar 实际安装角度
    roll = 20.0
    pitch = 0.0
    yaw = 90.0

    # imu 相对 radar 的平移
    extrinsic_T = np.array([-0.011, -0.02329, 0.04412], dtype=float)

    # 计算旋转矩阵
    R = euler_to_rotation_matrix(roll, pitch, yaw, order="ZYX")

    # 构造 4x4 外参矩阵
    T = build_transform(R, extrinsic_T)

    np.set_printoptions(precision=6, suppress=True)

    print("Rotation matrix R:")
    print(R)

    print("\nTranslation vector t:")
    print(extrinsic_T)

    print("\nHomogeneous transform T:")
    print(T)