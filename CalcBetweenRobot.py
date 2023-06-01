# -*- coding: utf-8 -*-

import cv2
import numpy as np
import scipy.linalg


class CalcBetweenRobot:
    def __init__(self):

        self.T_cam2baseA = np.load("data/A/T_cam2base.npy")
        self.T_baseB2cam = np.load("data/B/T_base2cam.npy")

    def get_T_B2A(self):

        T_B2A = np.zeros((4, 4))

        T_B2A = np.dot(self.T_cam2baseA, self.T_baseB2cam)

        # T_B2A[0][2] = 0.0
        # T_B2A[1][2] = 0.0

        # T_B2A[2][0] = 0.0
        # T_B2A[2][1] = 0.0
        # T_B2A[2][2] = 1.0
        # T_B2A[2][3] = 0.0

        return T_B2A

    def get_T_A2World(self, T_B2A):

        T_World2A = np.zeros((4, 4))
        R = np.zeros((3, 3))

        R = scipy.linalg.sqrtm(T_B2A[0:3, 0:3])

        T_World2A[0:3, 0:3] = R.real

        # T_World2A[2][2] = 1.0

        T_World2A[0][3] = T_B2A[0][3] / 2
        T_World2A[1][3] = T_B2A[1][3] / 2
        T_World2A[2][3] = T_B2A[2][3] / 2
        T_World2A[3][3] = 1.0

        return np.linalg.inv(T_World2A)

    def get_T_B2World(self, T_B2A, T_A2World):

        T_B2World = np.zeros((4, 4))

        T_B2World = np.dot(T_A2World, T_B2A)

        return T_B2World

    def save_T(self, T_B2World, T_A2World):

        np.save("data/T_baseB2world", T_B2World.real)
        np.save("data/T_baseA2world", T_A2World.real)


def main():
    calc = CalcBetweenRobot()

    T_B2A = calc.get_T_B2A()
    T_A2World = calc.get_T_A2World(T_B2A)
    T_B2World = calc.get_T_B2World(T_B2A, T_A2World)
    calc.save_T(T_B2World, T_A2World)

    print(T_B2A)
    print(np.dot(np.linalg.inv(T_A2World), T_B2World))

    print("fin!")


if __name__ == "__main__":
    main()
