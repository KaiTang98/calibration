# -*- coding: utf-8 -*-

import csv
import math
import random

import numpy as np

from utils import VS068_Kinematics as K


class random_generator:
    def __init__(self):

        self.DEG2RAD = math.pi / 180.0
        self.RAD2DEG = 180.0 / math.pi

        # 保存枚数
        self.save_num = 100

        # 手先移動量の最大
        self.max_trans = 7.0
        self.max_rot = 5.0

        # 現在の関節角の読み込み
        self.robot_name = "B"
        cur_angle_deg = np.loadtxt(
            "./data/" + self.robot_name + "/current_angle.csv"
        ).astype(np.float32)

        self.kine = K.KINE()

        # # 逆運動学の解選択に使用
        # self.kine.near_angle = [
        #     38.55 * self.DEG2RAD,
        #     25.87 * self.DEG2RAD,
        #     47.36 * self.DEG2RAD,
        #     77.0 * self.DEG2RAD,
        #     53.21 * self.DEG2RAD,
        #     -13.85 * self.DEG2RAD,
        # ]

        # 逆運動学の解選択に使用
        # 関節角を読み込んで使う時用
        self.kine.near_angle = [
            cur_angle_deg[0] * self.DEG2RAD,
            cur_angle_deg[1] * self.DEG2RAD,
            cur_angle_deg[2] * self.DEG2RAD,
            cur_angle_deg[3] * self.DEG2RAD,
            cur_angle_deg[4] * self.DEG2RAD,
            cur_angle_deg[5] * self.DEG2RAD,
        ]

        # エクステンション指定
        self.kine.Extension.x = -30.5
        self.kine.Extension.z = 65.5

        # 基準関節角度の指定_A
        # self.kine.Angle[0] = -36.88 * self.DEG2RAD
        # self.kine.Angle[1] = 24.47 * self.DEG2RAD
        # self.kine.Angle[2] = 50.87 * self.DEG2RAD
        # self.kine.Angle[3] = -79.25 * self.DEG2RAD
        # self.kine.Angle[4] = 54.49 * self.DEG2RAD
        # self.kine.Angle[5] = 20.4 * self.DEG2RAD
        # 基準関節角度の指定_B
        # self.kine.Angle[0] = 38.55 * self.DEG2RAD
        # self.kine.Angle[1] = 25.87 * self.DEG2RAD
        # self.kine.Angle[2] = 47.36 * self.DEG2RAD
        # self.kine.Angle[3] = 77.0 * self.DEG2RAD
        # self.kine.Angle[4] = 53.21 * self.DEG2RAD
        # self.kine.Angle[5] = -13.85 * self.DEG2RAD

        # 基準関節角度の指定_ファイルから読み込むとき
        self.kine.Angle[0] = cur_angle_deg[0] * self.DEG2RAD
        self.kine.Angle[1] = cur_angle_deg[1] * self.DEG2RAD
        self.kine.Angle[2] = cur_angle_deg[2] * self.DEG2RAD
        self.kine.Angle[3] = cur_angle_deg[3] * self.DEG2RAD
        self.kine.Angle[4] = cur_angle_deg[4] * self.DEG2RAD
        self.kine.Angle[5] = cur_angle_deg[5] * self.DEG2RAD

    def get_init_pose(self):

        # 基準位置格納変数
        init_pose = np.zeros(6)

        # 基準位置を求める
        self.kine.Forward()

        # 基準位置を格納する
        init_pose[0] = np.copy(self.kine.T[0, 3])
        init_pose[1] = np.copy(self.kine.T[1, 3])
        init_pose[2] = np.copy(self.kine.T[2, 3])
        init_pose[3] = self.kine.Euler.x * 180 / math.pi
        init_pose[4] = self.kine.Euler.y * 180 / math.pi
        init_pose[5] = self.kine.Euler.z * 180 / math.pi

        return init_pose

    def get_random(self):

        # ----------------変数定義----------------

        # 乱数保存用メモリ
        trans_rand = np.zeros(3)
        rot_rand = np.zeros(3)

        # ----------------乱数を生成する----------------

        for t_axis in range(0, 3):
            trans_rand[t_axis] = random.uniform(
                -self.max_trans, self.max_trans
            )
        for r_axis in range(0, 3):
            rot_rand[r_axis] = random.uniform(-self.max_rot, self.max_rot)

        return trans_rand, rot_rand

    def get_pose_angle(self, init_pose):

        # ----------------変数定義----------------

        # 計算結果保存用メモリ
        pose = np.zeros((1, 6))
        angle = np.zeros((1, 6))

        # 全結果保存用メモリ
        all_pose = np.empty((0, 6))
        all_T = np.empty((0, 4, 4))
        all_angle = np.empty((0, 6))

        # 取得した乱数を格納するメモリ
        trand = np.zeros(3)
        rrand = np.zeros(3)

        # ----------------計算部分----------------

        # 生成乱数に基づいて繰り返し，ランダムな角度位置を計算する
        for num in range(0, self.save_num):

            # 乱数を取得
            trand, rrand = self.get_random()

            # 基準位置に乱数で生成した微少移動分を足す
            pose[:, 0] = init_pose[0] + trand[0]
            pose[:, 1] = init_pose[1] + trand[1]
            pose[:, 2] = init_pose[2] + trand[2]
            pose[:, 3] = init_pose[3] + rrand[0]
            pose[:, 4] = init_pose[4] + rrand[1]
            pose[:, 5] = init_pose[5] + rrand[2]

            # 順逆用変数に渡す
            self.kine.T[0, 3] = np.copy(pose[:, 0])
            self.kine.T[1, 3] = np.copy(pose[:, 1])
            self.kine.T[2, 3] = np.copy(pose[:, 2])
            self.kine.Euler.x = pose[:, 3] * math.pi / 180
            self.kine.Euler.y = pose[:, 4] * math.pi / 180
            self.kine.Euler.z = pose[:, 5] * math.pi / 180

            # オイラー角から行列Rにする
            self.kine.InverseEuler()

            # 逆運動学を解く
            self.kine.Inverse()

            # 6次元の位置姿勢を保存用メモリにappendする
            all_pose = np.append(all_pose, pose, axis=0)

            # 6次元の位置姿勢を保存用メモリにappendする
            all_T = np.append(
                all_T, np.expand_dims(self.kine.T, axis=0), axis=0
            )

            for j in range(0, 6):
                angle[:, j] = self.kine.Angle[j] * 180 / math.pi

            # 関節角度を保存用メモリにappendする
            all_angle = np.append(all_angle, angle, axis=0)

        return all_pose, all_T, all_angle

    def file_save(self, all_pose, all_T, all_angle):

        with open("data/pose_" + str(self.save_num) + ".csv", "w") as f:
            writer = csv.writer(f, lineterminator="\n")  # 改行コード（\n）を指定しておく
            writer.writerows(all_pose)

        with open("data/pose_T_" + str(self.save_num) + ".csv", "w") as f:
            writer = csv.writer(f, lineterminator="\n")  # 改行コード（\n）を指定しておく
            writer.writerows(all_T)

        for i in range(0, all_T.shape[0]):
            with open("data/poses" + "%02d" % i + ".txt", "w") as f:
                writer = csv.writer(
                    f, lineterminator="\n", delimiter=" "
                )  # 改行コード（\n）を指定しておく
                writer.writerows(all_T[i, :, :])

        with open("data/angle_" + str(self.save_num) + ".csv", "w") as f:
            writer = csv.writer(f, lineterminator="\n")  # 改行コード（\n）を指定しておく
            writer.writerows(all_angle)


def main():

    rg = random_generator()

    # 基準位置姿勢を取得
    init_p = rg.get_init_pose()

    # ランダムな角度位置姿勢を一括取得
    pose, T, angle = rg.get_pose_angle(init_p)

    # 保存
    rg.file_save(pose, T, angle)

    print("fin!")


if __name__ == "__main__":

    main()
