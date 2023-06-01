import ctypes

import numpy as np

from utils import VS068_Kinematics as K


def get_robot_angle_A():
    # なぜか絶対パスでwinmode=0x8じゃないとダメ
    absolute_lib_path_A = R"C:\Users\Sapphire\Desktop\Kai\Controller_Ver.5.0.6\Themes\Sewing_VS\PositionBase\PyVS_A\x64\Release\PyVS_A.dll"
    PyVS_A = ctypes.WinDLL(absolute_lib_path_A, winmode=0x8)
    # PyVS_A = ctypes.WinDLL("./PyVS_A/x64/Release/PyVS_A.dll", winmode=1)
    PyInitSM_A = PyVS_A.PyInitSM
    PyInitSM_A.restype = ctypes.c_int

    PyReadSM_A = PyVS_A.PyReadSM
    PyReadSM_A.argtype = ctypes.POINTER(ctypes.c_double)

    CurPos_A = np.zeros((6), np.float64)

    ret = PyInitSM_A()

    PyReadSM_A(CurPos_A.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))

    print(f"Current angle A :{CurPos_A}")

    return CurPos_A


def get_robot_angle_B():
    absolute_lib_path_B = R"C:\Users\Sapphire\Desktop\Kai\Controller_Ver.5.0.6\Themes\Sewing_VS\PositionBase\PyVS_B\x64\Release\PyVS_B.dll"
    PyVS_B = ctypes.WinDLL(absolute_lib_path_B, winmode=0x8)

    PyInitSM_B = PyVS_B.PyInitSM
    PyInitSM_B.restype = ctypes.c_int

    PyReadSM_B = PyVS_B.PyReadSM
    PyReadSM_B.argtype = ctypes.POINTER(ctypes.c_double)

    CurPos_B = np.zeros((6), np.float64)

    ret = PyInitSM_B()

    PyReadSM_B(CurPos_B.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))

    print(f"Current angle B :{CurPos_B}")

    return CurPos_B


def save_robot_angle_pose(angle_deg, robot_name):
    DEG2RAD = np.pi / 180.0
    RAD2DEG = 180.0 / np.pi

    robot_pose = np.zeros(6)
    angle_rad = angle_deg * DEG2RAD

    kine = K.KINE()
    kine.Angle[0] = angle_rad[0]
    kine.Angle[1] = angle_rad[1]
    kine.Angle[2] = angle_rad[2]
    kine.Angle[3] = angle_rad[3]
    kine.Angle[4] = angle_rad[4]
    kine.Angle[5] = angle_rad[5]

    kine.Forward()

    robot_pose[0] = np.copy(kine.T[0, 3])
    robot_pose[1] = np.copy(kine.T[1, 3])
    robot_pose[2] = np.copy(kine.T[2, 3])
    robot_pose[3] = kine.Euler.x * RAD2DEG
    robot_pose[4] = kine.Euler.y * RAD2DEG
    robot_pose[5] = kine.Euler.z * RAD2DEG

    print(f"Robot pose {robot_name}:{robot_pose}")

    np.savetxt(
        "./data/" + robot_name + "/current_angle.csv", angle_deg, delimiter=","
    )
    np.savetxt(
        "./data/" + robot_name + "/current_pose.csv", robot_pose, delimiter=","
    )


def main():
    robot_A = "A"
    robot_B = "B"

    # cur_angle_A = get_robot_angle_A()
    # save_robot_angle_pose(cur_angle_A, robot_A)

    cur_angle_B = get_robot_angle_B()
    save_robot_angle_pose(cur_angle_B, robot_B)


if __name__ == "__main__":
    main()
