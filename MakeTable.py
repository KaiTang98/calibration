# -*- coding: utf-8 -*-
"""
Created on Fri May 21 08:40:30 2021

@author: root
"""

import cv2
import numpy as np

import config


class MakeTable:
    def __init__(self):
        self.rvecs_plane2cam = np.load("data/plane/rvecs_plane.npy")
        self.tvecs_plane2cam = np.load("data/plane/tvecs_plane.npy")
        self.T_plane2cam = np.zeros((4, 4), np.float64)
        self.T_plane2cam[0:3, 0:3], jac = cv2.Rodrigues(self.rvecs_plane2cam)
        self.T_plane2cam[0:3, 3:4] = self.tvecs_plane2cam
        self.T_plane2cam[3, 3] = 1

        self.T_cam2plane = np.linalg.inv(self.T_plane2cam)
        self.tvecs_cam2plane = self.T_cam2plane[0:3, 3:4]
        self.rvecs_cam2plane, jac = cv2.Rodrigues(self.T_cam2plane[0:3, 0:3])

        #       self.T_plane2cam = np.linalg.inv(self.T_cam2plane)

        # self.KK = np.array([[8680.2, 0, 570.9], [0, 8666.1, 479.7], [0, 0, 1]])
        self.KK = np.load("../Calibration/data/A/mtx.npy")
        self.dist = np.load("../Calibration/data/A/dist.npy")

        self.T_cam2baseA = np.load("data/A/T_cam2base.npy")
        self.T_target2cam = np.load("data/plane/T_target2cam.npy")
        self.T_baseA2world = np.load("data/T_baseA2world.npy")
        self.board = []
        self.base = []
        self.world = []

        self.width = config.WIDTH
        self.height = config.HEIGHT

        # print(self.T_cam2baseA)

    def Image2Board(self, corners2):
        Rmat, jac = cv2.Rodrigues(self.rvecs_plane2cam)
        R_inv = np.linalg.inv(Rmat)
        KK_inv = np.linalg.inv(self.KK)

        print("tvecs_plane2cam", self.tvecs_plane2cam)
        self.tmp = np.zeros((len(corners2), 1, 1, 3), np.float64)

        for i in range(len(corners2)):
            uv = cv2.convertPointsToHomogeneous(corners2[i])
            uv = uv.reshape(3, -1)
            temp1 = R_inv @ KK_inv @ uv
            temp2 = R_inv @ self.tvecs_plane2cam
            s = temp2[2]
            s /= temp1[2]
            temp3 = R_inv @ (s * KK_inv @ uv - self.tvecs_plane2cam)
            temp3 = temp3.reshape(1, 1, -1)

            self.tmp[i] = temp3
            if i % config.WIDTH == 0:
                print(
                    "\r{0}/{1}, {2:.1f} % calc".format(
                        i, len(corners2), (i / len(corners2) * 100)
                    ),
                    end="",
                )
        self.board = self.tmp.reshape(-1, 1, 3)

        # board2save = np.zeros((1255968, 4, 1), np.float64)
        board2save = np.zeros((777504, 4, 1), np.float64)
        for i in range(len(self.board)):
            #        for i in range(5):
            temp1 = cv2.convertPointsToHomogeneous(self.board[i])
            temp2 = temp1.reshape(4, 1)
            board2save[i] = temp2

        self.board_reshape = board2save.reshape(1246, 624, 4, 1)

        np.save("./data/table/board_undist", self.board_reshape)

    def Board2Base(self):
        for i in self.board:
            temp1 = cv2.convertPointsToHomogeneous(i)
            temp2 = temp1.reshape(4, 1)
            self.base.append(self.T_cam2baseA @ self.T_plane2cam @ temp2)

        self.np_base = np.asarray(self.base, np.float64)
        # print(np_Table)
        base_reshape = self.np_base.reshape(self.width, self.height, 4, 1)
        np.save("data/table/base_undist", base_reshape)

    def Base2World(self):
        for i in self.np_base:
            # print(i.shape)
            self.world.append(self.T_baseA2world @ i)

        np_world = np.asarray(self.world, np.float64)
        # Teble_reshape = np_world.reshape(1246, 1008, 4, 1)
        Teble_reshape = np_world.reshape(1246, 624, 4, 1)
        np.save("data/table/Table_img2world", Teble_reshape)


def main():
    size = (config.WIDTH, config.HEIGHT)
    pix = np.zeros((size[0] * size[1], 1, 2), np.float64)
    pix_undist = np.zeros((size[0] * size[1], 1, 2), np.float64)

    loop = 0

    table = MakeTable()

    for y_pix in range(size[0]):
        for x_pix in range(size[1]):
            # r = (x_pix + 1) ** 2 + (y_pix + 1) ** 2
            # x_undist = (x_pix + 1) * (
            #     1
            #     + table.dist[0][0] * r**2
            #     + table.dist[0][1] * r**4
            #     + table.dist[0][4] * r**6
            # )
            # y_undist = (y_pix + 1) * (
            #     1
            #     + table.dist[0][0] * r**2
            #     + table.dist[0][1] * r**4
            #     + table.dist[0][4] * r**6
            # )
            # pix_undist[loop, 0] = (y_undist, x_undist)
            pix[loop, 0] = (y_pix + 1, x_pix + 1)
            loop += 1

    print("init")
    table.Image2Board(pix)
    print("I2B")
    table.Board2Base()
    print(table.base[0])
    print("B2W")
    table.Base2World()
    print(table.dist)


if __name__ == "__main__":
    main()
