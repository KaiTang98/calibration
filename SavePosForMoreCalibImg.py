import ctypes
import csv
import math
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
        
        # なぜか絶対パスでwinmode=0x8じゃないとダメ
        absolute_lib_path_B = R"C:\Users\Sapphire\Desktop\Kai\Controller_Ver.5.0.6\Themes\Sewing_VS\PositionBase\PyVS_B\x64\Release\PyVS_B.dll"
        PyVS_B = ctypes.WinDLL(absolute_lib_path_B, winmode=0x8)
        # PyVS_A = ctypes.WinDLL("./PyVS_A/x64/Release/PyVS_A.dll", winmode=1)
        PyInitSM_B = PyVS_B.PyInitSM
        PyInitSM_B.restype = ctypes.c_int

        PyReadSM_B = PyVS_B.PyReadSM
        PyReadSM_B.argtype = ctypes.POINTER(ctypes.c_double)

        CurPos_B = np.zeros((6), np.float64)

        ret = PyInitSM_B()

        PyReadSM_B(CurPos_B.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))

        print(f"Current angle B :{CurPos_B}")

        return CurPos_B


class get_robot_state:
    
    def __init__(self):

        self.DEG2RAD = math.pi / 180.0
        self.RAD2DEG = 180.0 / math.pi

        self.kine = K.KINE()
    
    def get_init_pose(self, cur_angle):

        self.kine.near_angle = [
            cur_angle[0] * self.DEG2RAD,
            cur_angle[1] * self.DEG2RAD,
            cur_angle[2] * self.DEG2RAD,
            cur_angle[3] * self.DEG2RAD,
            cur_angle[4] * self.DEG2RAD,
            cur_angle[5] * self.DEG2RAD,
        ]

        self.kine.Angle[0] = cur_angle[0] * self.DEG2RAD
        self.kine.Angle[1] = cur_angle[1] * self.DEG2RAD
        self.kine.Angle[2] = cur_angle[2] * self.DEG2RAD
        self.kine.Angle[3] = cur_angle[3] * self.DEG2RAD
        self.kine.Angle[4] = cur_angle[4] * self.DEG2RAD
        self.kine.Angle[5] = cur_angle[5] * self.DEG2RAD

        # initialize init pose
        init_pose = np.zeros(6)

        # forward kinematics
        self.kine.Forward()

        init_pose[0] = np.copy(self.kine.T[0, 3])
        init_pose[1] = np.copy(self.kine.T[1, 3])
        init_pose[2] = np.copy(self.kine.T[2, 3])
        init_pose[3] = self.kine.Euler.x * 180 / math.pi
        init_pose[4] = self.kine.Euler.y * 180 / math.pi
        init_pose[5] = self.kine.Euler.z * 180 / math.pi

        return init_pose
    

    def get_rest_state(self, init_pose):

        pose = np.zeros((1, 6))
        angle = np.zeros((1, 6))


        pose[:, 0] = init_pose[0]
        pose[:, 1] = init_pose[1]
        pose[:, 2] = init_pose[2]
        pose[:, 3] = init_pose[3]
        pose[:, 4] = init_pose[4]
        pose[:, 5] = init_pose[5]

        self.kine.T[0, 3] = np.copy(pose[:, 0])
        self.kine.T[1, 3] = np.copy(pose[:, 1])
        self.kine.T[2, 3] = np.copy(pose[:, 2])
        self.kine.Euler.x = pose[:, 3] * math.pi / 180
        self.kine.Euler.y = pose[:, 4] * math.pi / 180
        self.kine.Euler.z = pose[:, 5] * math.pi / 180

        T = np.expand_dims(self.kine.T, axis=0)

        for j in range(0, 6):
                angle[:, j] = self.kine.Angle[j] * 180 / math.pi

        return pose, T, angle
    
    def save_state(self, pose, T, angle):

        with open("data/B/pose_100.csv", "a") as f:
            writer = csv.writer(f, lineterminator="\n")  # 改行コード（\n）を指定しておく
            writer.writerows(pose)

        with open("data/B/pose_T_100.csv", "a") as f:
            writer = csv.writer(f, lineterminator="\n")  # 改行コード（\n）を指定しておく
            writer.writerows(T)

        for i in range(0, T.shape[0]):
            with open("data/poses.txt", "w") as f:
                writer = csv.writer(f, lineterminator="\n", delimiter=" ")
                writer.writerows(T[i, :])

        with open("data/B/angle_100.csv", "a") as f:
            writer = csv.writer(f, lineterminator="\n")  # 改行コード（\n）を指定しておく
            writer.writerows(angle)    

def main():
    
    robot_A = "A"
    robot_B = "B"

    get_state = get_robot_state()

    # cur_angle_A = get_robot_angle_A()

    # init_pos = get_state.get_init_pose(cur_angle_A)

    # pose, T, angle = get_state.get_rest_state(init_pos)

    # print(pose)
    # print(T)
    # print(angle)
    
    # get_state.save_state(pose, T, angle)


    cur_angle_B = get_robot_angle_B()

    init_pos = get_state.get_init_pose(cur_angle_B)

    pose, T, angle = get_state.get_rest_state(init_pos)

    print(pose)
    print(T)
    print(angle)
    
    get_state.save_state(pose, T, angle)

    # save_robot_angle_pose(cur_angle_A, robot_A)

    # cur_angle_B = get_robot_angle_B()
    # save_robot_angle_pose(cur_angle_B, robot_B)

    # for debug
    # cur_angle_deg = np.loadtxt(
    #         "./data/A/poses100.txt"
    #     ).astype(np.float32)

    # init_pos = get_state.get_init_pose(cur_angle_deg)

    # pose, T, angle = get_state.get_rest_state(init_pos)

    # print(pose)
    # print(T)
    # print(angle)
    

if __name__ == "__main__":
    main()
