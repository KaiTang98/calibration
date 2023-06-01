#!/usr/bin/env python
# coding: utf-8


import glob
import time

import cv2
import cv2 as cv
import numpy as np
import pypuclib
from pypuclib import (PUC_DATA_MODE, Camera, CameraFactory, Decoder,
                      PUCException, Resolution, XferData)


class plane_estimation:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.w = 1246
        self.h = 624

        self.square_size = 1.5  # 正方形の1辺のサイズ[cm]
        self.pattern_size = (3, 4)  # 交差ポイントの数(チェッカーのマスの数ではない)

        self.pattern_points = np.zeros(
            (np.prod(self.pattern_size), 3), np.float64
        )
        self.pattern_points[:, :2] = np.indices(self.pattern_size).T.reshape(
            -1, 2
        )
        self.pattern_points *= self.square_size

        # 推定した位置姿勢を分かりやすく可視化するための仮想的な物体
        self.cube = np.float64(
            [
                [0, 0, 0],
                [0, 3, 0],
                [3, 3, 0],
                [3, 0, 0],
                [0, 0, -3],
                [0, 3, -3],
                [3, 3, -3],
                [3, 0, -3],
            ]
        )

        self.mtx = np.load("../Calibration/data/A/mtx.npy")
        self.dist = np.load("../Calibration/data/A/dist.npy")

    def save_plane_coordinate(self):

        cam = CameraFactory().create()
        cam.setFramerateShutter(125, 125)
        decoder = cam.decoder()

        loop = 0

        while True:
            xferData = cam.grab()

            # Decode the data can be used as image
            frame = decoder.decode(xferData, self.x, self.y, self.w, self.h)

            # フレームを表示する
            cv2.imshow("captured image", frame)

            c_frame = np.copy(frame)

            ret, corners = cv.findChessboardCorners(
                frame, self.pattern_size, None
            )

            if ret == True:

                # 座標の精度を上げる
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT,
                    100,
                    0.001,
                )
                corners = cv.cornerSubPix(
                    frame, corners, (5, 5), (-1, -1), criteria
                )

                # solvePnP を用いて外部パラメータ (物体の位置姿勢に相当) だけを推定します。
                err, rvecs, tvecs = cv.solvePnP(
                    self.pattern_points, corners, self.mtx, self.dist
                )

                # 推定結果を可視化するために、物体 cube を画像に投影してみます。
                imgpts, jac = cv.projectPoints(
                    self.cube, rvecs, tvecs, self.mtx, self.dist
                )

                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

                c_frame = self.draw(frame, imgpts)

                if loop % 100 == 0:
                    # カメラ作業台の同次変換行列
                    T_ext = np.zeros((4, 4), np.float64)

                    R_ext, jac = cv2.Rodrigues(rvecs)

                    T_ext[0:3, 0:3] = R_ext
                    T_ext[0:3, 3:4] = tvecs[0:3]
                    T_ext[3][3] = 1.0

                    print("T\n", T_ext)

            cv2.imshow("plane coordinate", c_frame)

            k = cv.waitKey(1)
            loop += 1

            if k == 27:
                break

        cv2.destroyAllWindows()

        np.save("data/plane/rvecs_plane", rvecs)
        np.save("data/plane/tvecs_plane", tvecs)

    def draw(self, img, imgpts):

        imgpts = np.int32(imgpts).reshape(-1, 2)

        # draw ground floor in green
        img = cv.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)

        # draw pillars in blue color
        for i, j in zip(range(4), range(4, 8)):
            img = cv.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 3)

        # draw top layer in red color
        img = cv.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 3)

        return img


def main():
    p_est = plane_estimation()
    p_est.save_plane_coordinate()


if __name__ == "__main__":
    main()
