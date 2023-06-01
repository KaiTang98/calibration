# -*- coding: utf-8 -*-
"""
Created on Mon Aug 28 17:46:07 2017

@author: Konada

Python Version : 3.5
"""

import math

import numpy as np

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

EPS = 3.0e-5
EPS_POS = 5.0e-3

# DH記法によるリンク長さ a[mm]
vs068_a = [30.0, 340.0, 20.0, 0.0, 0.0, 0.0]

# リンク間距離	d[mm]
vs068_d = [395.0, 0.0, 0.0, 340.0, 0.0, 80.0]

# VS068の仕様上のリミット
vs068_jangle_max = [170.0, 135.0, 153.0, 270.0, 120.0, 360.0]
vs068_jangle_min = [-170.0, -100.0, -120.0, -270.0, -120.0, -360.0]

# XYZのリミット	offset_xを足した後の値を入れてください。
vs068_XYZ_max = [1000.0, 1000.0, 0.0]
vs068_XYZ_min = [-1000.0, -1000.0, -1500.0]

# 関節角のリミット 適宜緩めてください。
angle_max_deg = [165.0, 130.0, 148.0, 265.0, 115.0, 360.0]
angle_min_deg = [-165.0, -95.0, -115.0, -265.0, -115.0, -360.0]

# XYZのリミット
XYZ_max = [1000.0, 1000.0, 0.0]
XYZ_min = [-1000.0, -1000.0, -1500.0]


class KINE:
    class Point3D:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    def __init__(self):
        # 逆運動学の結果を格納
        self.Angle = [
            0 * DEG2RAD,
            0 * DEG2RAD,
            90.0 * DEG2RAD,
            0 * DEG2RAD,
            0 * DEG2RAD,
            0 * DEG2RAD,
        ]

        # 全逆運動学の結果の格納
        self.angleResult_All = []
        for i in range(8):
            self.angleResult_All.append([])
            for j in range(6):
                self.angleResult_All[i].append(0.0)

        # ハンドのExtension
        self.Extension = KINE.Point3D()
        self.Extension.x = 0.0
        self.Extension.y = 0.0
        self.Extension.z = 0.0

        # 手先のxyzオイラー姿勢
        self.Euler = KINE.Point3D()
        self.Euler.x = 0.0
        self.Euler.y = 0.0
        self.Euler.z = 0.0

        # 逆運動学選択の際の一番近い関節角度
        #    str_des_angle = '108.86 -10.31 73.05 -38.68 30.67 -13.76'
        self.near_angle = [
            -36.0 * DEG2RAD,
            68.0 * DEG2RAD,
            80.0 * DEG2RAD,
            60.0 * DEG2RAD,
            -81.0 * DEG2RAD,
            -124.0 * DEG2RAD,
        ]

        # 逆運動学選択の際の重み
        self.angle_weight = [1.5, 5.0, 5.0, 1.0, 0.5, 0.5]

        # エラーフラグ
        self.errorFlag = []
        for i in range(8):
            self.errorFlag.append(-9)

        # 手先同時変換行列の格納
        self.T = np.zeros([4, 4])

        # 順運動学の計算
        self.Forward()

    def Forward(self):
        calc_T = np.zeros([4, 4])

        c1 = math.cos(self.Angle[0])
        c2 = math.cos(self.Angle[1])
        c3 = math.cos(self.Angle[2])
        c4 = math.cos(self.Angle[3])
        c5 = math.cos(self.Angle[4])
        c6 = math.cos(self.Angle[5])

        s1 = math.sin(self.Angle[0])
        s2 = math.sin(self.Angle[1])
        s3 = math.sin(self.Angle[2])
        s4 = math.sin(self.Angle[3])
        s5 = math.sin(self.Angle[4])
        s6 = math.sin(self.Angle[5])

        # リンク長さ
        a0 = vs068_a[0]
        a1 = vs068_a[1]
        a2 = vs068_a[2]

        d1 = vs068_d[0]
        d4 = vs068_d[3]
        d6 = vs068_d[5]

        p0 = -s3 * (c4 * c5 * c6 - s4 * s6) - c3 * c6 * s5
        p1 = c3 * (c4 * c5 * c6 - s4 * s6) - c6 * s3 * s5
        p2 = c4 * s6 + c5 * c6 * s4
        calc_T[0][0] = c1 * (s2 * p0 + c2 * p1) - s1 * p2
        calc_T[1][0] = s1 * (s2 * p0 + c2 * p1) + c1 * p2
        calc_T[2][0] = c2 * p0 - s2 * p1

        p0 = -s3 * (-c4 * c5 * s6 - c6 * s4) + c3 * s5 * s6
        p1 = c3 * (-c4 * c5 * s6 - c6 * s4) + s3 * s5 * s6
        p2 = c4 * c6 - c5 * s4 * s6
        calc_T[0][1] = c1 * (s2 * p0 + c2 * p1) - s1 * p2
        calc_T[1][1] = s1 * (s2 * p0 + c2 * p1) + c1 * p2
        calc_T[2][1] = c2 * p0 - s2 * p1

        p0 = -c4 * s3 * s5 + c3 * c5
        p1 = c3 * c4 * s5 + c5 * s3
        calc_T[0][2] = c1 * (s2 * p0 + c2 * p1) - s1 * s4 * s5
        calc_T[1][2] = s1 * (s2 * p0 + c2 * p1) + c1 * s4 * s5
        calc_T[2][2] = c2 * p0 - s2 * p1

        # 位置を計算
        p0 = -c4 * d6 * s3 * s5 + a2 * s3 + c3 * (c5 * d6 + d4)
        p1 = c3 * c4 * d6 * s5 + (c5 * d6 + d4) * s3 - a2 * c3
        calc_T[0][3] = (
            c1 * (s2 * p0 + c2 * p1 + a1 * s2) - d6 * s1 * s4 * s5 + a0 * c1
        )
        calc_T[1][3] = (
            s1 * (s2 * p0 + c2 * p1 + a1 * s2) + c1 * d6 * s4 * s5 + a0 * s1
        )
        calc_T[2][3] = c2 * p0 - s2 * p1 + d1 + a1 * c2

        # extensionを考慮
        calc_T[0][3] += (
            self.Extension.x * calc_T[0][0]
            + self.Extension.y * calc_T[0][1]
            + self.Extension.z * calc_T[0][2]
        )
        calc_T[1][3] += (
            self.Extension.x * calc_T[1][0]
            + self.Extension.y * calc_T[1][1]
            + self.Extension.z * calc_T[1][2]
        )
        calc_T[2][3] += (
            self.Extension.x * calc_T[2][0]
            + self.Extension.y * calc_T[2][1]
            + self.Extension.z * calc_T[2][2]
        )

        calc_T[3][3] = 1

        # ワールド座標に変換 and 変数へ出力
        rote_T = [[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
        for i in range(4):
            for j in range(4):
                # out_T[i][j] = calc_T[i][j]
                self.T[i][j] = (
                    rote_T[i][0] * calc_T[0][j]
                    + rote_T[i][1] * calc_T[1][j]
                    + rote_T[i][2] * calc_T[2][j]
                    + rote_T[i][3] * calc_T[3][j]
                )

        # オイラー角変換
        self.Euler.z = math.atan2(-self.T[0][1], self.T[0][0])
        self.Euler.y = math.asin(self.T[0][2])
        self.Euler.x = math.atan2(-self.T[1][2], self.T[2][2])

        if self.Euler.x > math.pi:
            self.Euler.x -= 2 * math.pi
        elif self.Euler.x < -math.pi:
            self.Euler.x += 2 * math.pi

        if self.Euler.y > math.pi:
            self.Euler.y -= 2 * math.pi
        elif self.Euler.y < -math.pi:
            self.Euler.y += 2 * math.pi

        if self.Euler.z > math.pi:
            self.Euler.z -= 2 * math.pi
        elif self.Euler.z < -math.pi:
            self.Euler.z += 2 * math.pi

        for i in range(8):
            for axis in range(6):
                self.angleResult_All[i][axis] = self.Angle[axis]

    def Inverse(self):
        calc_T = np.zeros([4, 4])

        a0 = vs068_a[0]
        a1 = vs068_a[1]
        a2 = vs068_a[2]

        d0 = vs068_d[0]
        d3 = vs068_d[3]
        d5 = vs068_d[5]

        c0 = []
        s0 = []
        c1 = []
        s1 = []
        c2 = []
        s2 = []
        c4 = []

        for i in range(8):
            c0.append(0.0)
            s0.append(0.0)
            c1.append(0.0)
            s1.append(0.0)
            c2.append(0.0)
            s2.append(0.0)
            c4.append(0.0)

        w = KINE.Point3D()
        wp = KINE.Point3D()
        wpp = KINE.Point3D()
        wppp = KINE.Point3D()

        # エラーフラグのリセット
        for i in range(8):
            self.errorFlag[i] = 1

        ############################################################################
        # ワールドからロボット座標系に変換
        ############################################################################
        rote_T = [[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
        for i in range(4):
            for j in range(4):
                calc_T[i][j] = (
                    rote_T[i][0] * self.T[0, j]
                    + rote_T[i][1] * self.T[1, j]
                    + rote_T[i][2] * self.T[2, j]
                    + rote_T[i][3] * self.T[3, j]
                )

        ############################################################################
        # extensionを考慮
        ############################################################################
        calc_T[0][3] -= (
            self.Extension.x * calc_T[0][0]
            + self.Extension.y * calc_T[0][1]
            + self.Extension.z * calc_T[0][2]
        )
        calc_T[1][3] -= (
            self.Extension.x * calc_T[1][0]
            + self.Extension.y * calc_T[1][1]
            + self.Extension.z * calc_T[1][2]
        )
        calc_T[2][3] -= (
            self.Extension.x * calc_T[2][0]
            + self.Extension.y * calc_T[2][1]
            + self.Extension.z * calc_T[2][2]
        )

        ############################################################################
        # 第1関節
        ############################################################################
        # 手首の位置ベクトルを計算
        w.x = calc_T[0][3] - d5 * calc_T[0][2]
        w.y = calc_T[1][3] - d5 * calc_T[1][2]
        w.z = calc_T[2][3] - d5 * calc_T[2][2]

        for i in range(4):
            self.angleResult_All[i][0] = math.atan2(w.y, w.x)

        for i in range(4, 8):
            self.angleResult_All[i][0] = math.atan2(-w.y, -w.x)

        # 角度を-math.piからmath.piに変換
        for i in range(8):
            if self.angleResult_All[i][0] > math.pi:
                self.angleResult_All[i][0] -= 2 * math.pi

            elif self.angleResult_All[i][0] < -math.pi:
                self.angleResult_All[i][0] += 2 * math.pi

        for i in range(8):
            s0[i] = math.sin(self.angleResult_All[i][0])
            c0[i] = math.cos(self.angleResult_All[i][0])

        ############################################################################
        # 第2関節 および 第3関節
        ############################################################################
        for i in [0, 4]:
            # 第1関節の角度が0degで、第2関節の中心をベース座標の原点とした場合の手首の位置ベクトルwpを計算
            wp.x = w.x * c0[i] + w.y * s0[i] - a0
            wp.y = w.z - d0
            wp.z = 0.0

            wpp.x = wp.x * wp.x + (wp.y + a1) * (wp.y + a1) - d3 * d3 - a2 * a2
            wpp.y = -2.0 * a1 * wp.x
            wpp.z = wp.x * wp.x + (wp.y - a1) * (wp.y - a1) - d3 * d3 - a2 * a2

            wppp.x = (
                wp.x * wp.x + wp.y * wp.y - (d3 - a1) * (d3 - a1) - a2 * a2
            )
            wppp.y = 2.0 * a1 * a2
            wppp.z = (
                wp.x * wp.x + wp.y * wp.y - (d3 + a1) * (d3 + a1) - a2 * a2
            )

            # 第2関節,第3関節に関する二次方程式の解がないとき
            if (
                wpp.y * wpp.y - wpp.x * wpp.z < 0
                or wppp.y * wppp.y - wppp.x * wppp.z < 0
            ):
                for j in range(4):
                    self.errorFlag[i + j] = -1
                    self.angleResult_All[i + j][1] = 0.0
            else:
                # 第2関節,第3関節の角度を計算
                for j in range(2):
                    self.angleResult_All[i + j][1] = 2.0 * math.atan2(
                        -wpp.y - math.sqrt(wpp.y * wpp.y - wpp.x * wpp.z),
                        wpp.x,
                    )
                    self.angleResult_All[i + j][2] = -2.0 * math.atan2(
                        -wppp.y - math.sqrt(wppp.y * wppp.y - wppp.x * wppp.z),
                        wppp.x,
                    )
                for j in range(2, 4):
                    self.angleResult_All[i + j][1] = 2.0 * math.atan2(
                        -wpp.y + math.sqrt(wpp.y * wpp.y - wpp.x * wpp.z),
                        wpp.x,
                    )
                    self.angleResult_All[i + j][2] = -2.0 * math.atan2(
                        -wppp.y + math.sqrt(wppp.y * wppp.y - wppp.x * wppp.z),
                        wppp.x,
                    )

        ############################################################################
        # 第4関節, 第5関節, 第6関節
        ############################################################################
        for i in [0, 2, 4]:
            if self.errorFlag[i] == 1:
                s1[i] = math.sin(self.angleResult_All[i][1])
                c1[i] = math.cos(self.angleResult_All[i][1])

                s2[i] = math.sin(self.angleResult_All[i][2])
                c2[i] = math.cos(self.angleResult_All[i][2])

                c4[i] = s2[i] * (
                    c1[i] * (calc_T[1][2] * s0[i] + calc_T[0][2] * c0[i])
                    - calc_T[2][2] * s1[i]
                ) + c2[i] * (
                    s1[i] * (calc_T[1][2] * s0[i] + calc_T[0][2] * c0[i])
                    + calc_T[2][2] * c1[i]
                )

                self.angleResult_All[i][4] = math.acos(c4[i])
                self.angleResult_All[i][3] = math.atan2(
                    calc_T[1][2] * c0[i] - calc_T[0][2] * s0[i],
                    c2[i]
                    * (
                        c1[i] * (calc_T[1][2] * s0[i] + calc_T[0][2] * c0[i])
                        - calc_T[2][2] * s1[i]
                    )
                    - s2[i]
                    * (
                        s1[i] * (calc_T[1][2] * s0[i] + calc_T[0][2] * c0[i])
                        + calc_T[2][2] * c1[i]
                    ),
                )
                self.angleResult_All[i][5] = math.pi + math.atan2(
                    -(
                        s2[i]
                        * (
                            c1[i]
                            * (calc_T[1][1] * s0[i] + calc_T[0][1] * c0[i])
                            - calc_T[2][1] * s1[i]
                        )
                        + c2[i]
                        * (
                            s1[i]
                            * (calc_T[1][1] * s0[i] + calc_T[0][1] * c0[i])
                            + calc_T[2][1] * c1[i]
                        )
                    ),
                    (
                        s2[i]
                        * (
                            c1[i]
                            * (calc_T[1][0] * s0[i] + calc_T[0][0] * c0[i])
                            - calc_T[2][0] * s1[i]
                        )
                        + c2[i]
                        * (
                            s1[i]
                            * (calc_T[1][0] * s0[i] + calc_T[0][0] * c0[i])
                            + calc_T[2][0] * c1[i]
                        )
                    ),
                )

                self.angleResult_All[i + 1][4] = -math.acos(c4[i])
                self.angleResult_All[i + 1][3] = math.pi + math.atan2(
                    calc_T[1][2] * c0[i] - calc_T[0][2] * s0[i],
                    c2[i]
                    * (
                        c1[i] * (calc_T[1][2] * s0[i] + calc_T[0][2] * c0[i])
                        - calc_T[2][2] * s1[i]
                    )
                    - s2[i]
                    * (
                        s1[i] * (calc_T[1][2] * s0[i] + calc_T[0][2] * c0[i])
                        + calc_T[2][2] * c1[i]
                    ),
                )
                self.angleResult_All[i + 1][5] = math.atan2(
                    -(
                        s2[i]
                        * (
                            c1[i]
                            * (calc_T[1][1] * s0[i] + calc_T[0][1] * c0[i])
                            - calc_T[2][1] * s1[i]
                        )
                        + c2[i]
                        * (
                            s1[i]
                            * (calc_T[1][1] * s0[i] + calc_T[0][1] * c0[i])
                            + calc_T[2][1] * c1[i]
                        )
                    ),
                    (
                        s2[i]
                        * (
                            c1[i]
                            * (calc_T[1][0] * s0[i] + calc_T[0][0] * c0[i])
                            - calc_T[2][0] * s1[i]
                        )
                        + c2[i]
                        * (
                            s1[i]
                            * (calc_T[1][0] * s0[i] + calc_T[0][0] * c0[i])
                            + calc_T[2][0] * c1[i]
                        )
                    ),
                )
            else:
                for j in range(2):
                    self.angleResult_All[i + j][3] = 0.0
                    self.angleResult_All[i + j][4] = 0.0
                    self.angleResult_All[i + j][5] = 0.0

        ############################################################################
        # 角度を-180から180にする
        ############################################################################
        for i in range(8):
            for j in range(6):
                if self.angleResult_All[i][j] > math.pi:
                    self.angleResult_All[i][j] -= 2 * math.pi
                if self.angleResult_All[i][j] < -math.pi:
                    self.angleResult_All[i][j] += 2 * math.pi

        ############################################################################
        # リミットチェック
        ############################################################################
        self.__LimitCheck()

        ############################################################################
        # 逆運動学の選択
        ############################################################################
        result = self.__GetOneInv()

        return result

    def InverseEuler(self):
        sx = math.sin(self.Euler.x)
        cx = math.cos(self.Euler.x)

        sy = math.sin(self.Euler.y)
        cy = math.cos(self.Euler.y)

        sz = math.sin(self.Euler.z)
        cz = math.cos(self.Euler.z)

        self.T[0, 0] = cy * cz
        self.T[0, 1] = -cy * sz
        self.T[0, 2] = sy

        self.T[1, 0] = sx * sy * cz + cx * sz
        self.T[1, 1] = -sx * sy * sz + cx * cz
        self.T[1, 2] = -sx * cy

        self.T[2, 0] = -cx * sy * cz + sx * sz
        self.T[2, 1] = cx * sy * sz + sx * cz
        self.T[2, 2] = cx * cy

    def __LimitCheck(self):
        ############################################################################
        # 逆運動学の選択
        ############################################################################
        for i in range(8):
            if self.errorFlag[i] == 1:
                # Xの範囲外 (user設定)
                if self.T[0, 3] < XYZ_min[0] or XYZ_max[0] < self.T[0, 3]:
                    self.errorFlag[i] = -2

                # Yの範囲外 (user設定)
                if self.T[1, 3] < XYZ_min[1] or XYZ_max[1] < self.T[1, 3]:
                    self.errorFlag[i] = -2

                # Zの範囲外 (user設定)
                if self.T[2, 3] < XYZ_min[2] or XYZ_max[2] < self.T[2, 3]:
                    self.errorFlag[i] = -2
                # Xの範囲外 (system設定)
                if (
                    self.T[0, 3] < vs068_XYZ_min[0]
                    or vs068_XYZ_max[0] < self.T[0, 3]
                ):
                    self.errorFlag[i] = -6

                # Yの範囲外 (system設定)
                if (
                    self.T[1, 3] < vs068_XYZ_min[1]
                    or vs068_XYZ_max[1] < self.T[1][3]
                ):
                    self.errorFlag[i] = -6

                # Zの範囲外 (system設定)
                if (
                    self.T[2, 3] < vs068_XYZ_min[2]
                    or vs068_XYZ_max[2] < self.T[2, 3]
                ):
                    self.errorFlag[i] = -6

                for j in range(6):
                    # 関節角が範囲外 (user設定)
                    if (
                        self.angleResult_All[i][j] < angle_min_deg[j] * DEG2RAD
                        or angle_max_deg[j] * DEG2RAD
                        < self.angleResult_All[i][j]
                    ):
                        self.errorFlag[i] = -3

                    # 関節角が範囲外 (system設定)
                    elif (
                        self.angleResult_All[i][j]
                        < vs068_jangle_min[j] * DEG2RAD
                        or vs068_jangle_max[j] * DEG2RAD
                        < self.angleResult_All[i][j]
                    ):
                        self.errorFlag[i] = -7

    def __GetOneInv(self):
        min_order = -1
        diff = 0.0
        min_diff = 360.0 * 6.0 * 10

        # nearアングルと近い関節角を見つける
        for i in range(8):
            if self.errorFlag[i] > 0:  # ちゃんと結果が出てたら差分を計算
                diff = 0
                for j in range(6):
                    # 重み行列をかけた差分を足す。２・３軸はあんま動いてほしくないよね？
                    diff += abs(
                        self.angle_weight[j]
                        * (self.angleResult_All[i][j] - self.near_angle[j])
                    )

                if min_diff > diff:  # 差分が小さければ値を更新
                    min_diff = diff
                    min_order = i
        # -1だったら全部失敗してる。
        if min_order != -1:
            for i in range(6):
                # 値を決定する。
                self.Angle[i] = self.angleResult_All[min_order][i]

        else:
            return -1

        return 0

    def ShowError(self):
        print("\n------------------------------\nエラーログ\n")
        for i in range(8):
            print(
                "\n%d:%8.3f  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f\n"
                % (
                    i + 1,
                    self.angleResult_All[i][0] * RAD2DEG,
                    self.angleResult_All[i][1] * RAD2DEG,
                    self.angleResult_All[i][2] * RAD2DEG,
                    self.angleResult_All[i][3] * RAD2DEG,
                    self.angleResult_All[i][4] * RAD2DEG,
                    self.angleResult_All[i][5] * RAD2DEG,
                )
            )

            if self.errorFlag[i] == 1:
                print("この値は使用することができます。\n")

            elif self.errorFlag[i] == -1:
                print("この一軸では逆運動学の解が存在しません。手の届かないところにあるか無理な姿勢になっています。\n")
                print(
                    "%8.3f %8.3f %8.3f %8.3f\n"
                    % (self.T[0][0], self.T[0][1], self.T[0][2], self.T[0][3])
                )
                print(
                    "%8.3f %8.3f %8.3f %8.3f\n"
                    % (self.T[1][0], self.T[1][1], self.T[1][2], self.T[1][3])
                )
                print(
                    "%8.3f %8.3f %8.3f %8.3f\n"
                    % (self.T[2][0], self.T[2][1], self.T[2][2], self.T[2][3])
                )
                print(
                    "%8.3f %8.3f %8.3f %8.3f\n"
                    % (self.T[3][0], self.T[3][1], self.T[3][2], self.T[3][3])
                )

            elif self.errorFlag[i] == -2:
                print(
                    "手先の座標が範囲外になっています。\n範囲を広げるにはkine.XYZ_max or minを編集してください。\n"
                )
                print(
                    "X=%8.3f MAX:%8.3f MIN:%8.3f\n"
                    % (self.T[0][3], XYZ_max[0], XYZ_min[0])
                )
                print(
                    "Y=%8.3f MAX:%8.3f MIN:%8.3f\n"
                    % (self.T[1][3], XYZ_max[1], XYZ_min[1])
                )
                print(
                    "Z=%8.3f MAX:%8.3f MIN:%8.3f\n"
                    % (self.T[2][3], XYZ_max[2], XYZ_min[2])
                )

            elif self.errorFlag[i] == -3:
                print(
                    "関節角が範囲外になっています。\n範囲を広げるにはkine.angle_max or minを編集してください。\n"
                )
                for j in range(6):
                    if (
                        self.angleResult_All[i][j] * RAD2DEG > angle_max_deg[j]
                        or self.angleResult_All[i][j] * RAD2DEG
                        < angle_min_deg[j]
                    ):
                        print(
                            "第%d軸:%8.3f MAX:%8.3f MIN:%8.3f\n"
                            % (
                                j + 1,
                                self.angleResult_All[i][j] * RAD2DEG,
                                angle_max_deg[j],
                                angle_min_deg[j],
                            )
                        )

            elif self.errorFlag[i] == -4:
                print("逆運動学の後に順運動学を解くと元に戻りません。\n同次変換行列の入力が間違っている可能性があります。\n")
                print(
                    "%8.3f %8.3f %8.3f %8.3f\n"
                    % (self.T[0][0], self.T[0][1], self.T[0][2], self.T[0][3])
                )
                print(
                    "%8.3f %8.3f %8.3f %8.3f\n"
                    % (self.T[1][0], self.T[1][1], self.T[1][2], self.T[1][3])
                )
                print(
                    "%8.3f %8.3f %8.3f %8.3f\n"
                    % (self.T[2][0], self.T[2][1], self.T[2][2], self.T[2][3])
                )
                print(
                    "%8.3f %8.3f %8.3f %8.3f\n"
                    % (self.T[3][0], self.T[3][1], self.T[3][2], self.T[3][3])
                )

            elif self.errorFlag[i] == -5:
                print("5軸が0度です。その姿勢は特異点なので逆運動学の解(4軸と6軸)が一つに定まりません\n")

            elif self.errorFlag[i] == -6:
                print("注意！！手先の座標が範囲外になっています。\n範囲を広げる前に本当に正しい値なのかよく吟味してください。\n")
                print("Warning！！not good XYZ,you have to Rethink\n")
                print(
                    "X=%8.3f MAX:%8.3f MIN:%8.3f\n"
                    % (self.T[0][3], vs068_XYZ_max[0], vs068_XYZ_min[0])
                )
                print(
                    "Y=%8.3f MAX:%8.3f MIN:%8.3f\n"
                    % (self.T[1][3], vs068_XYZ_max[1], vs068_XYZ_min[1])
                )
                print(
                    "Z=%8.3f MAX:%8.3f MIN:%8.3f\n"
                    % (self.T[2][3], vs068_XYZ_max[2], vs068_XYZ_min[2])
                )

            elif self.errorFlag[i] == -7:
                print("注意！！関節角が範囲外になっています。\n範囲を広げる前に本当に正しい値なのかよく吟味してください。\n")
                print("Warning！！not good angle,you have to Rethink\n")
                for j in range(6):
                    if (
                        self.angleResult_All[i][j] * RAD2DEG
                        > vs068_jangle_max[j]
                        or self.angleResult_All[i][j] * RAD2DEG
                        < vs068_jangle_min[j]
                    ):
                        print(
                            "第%d軸:%8.3f MAX:%8.3f MIN:%8.3f\n"
                            % (
                                j + 1,
                                self.angleResult_All[i][j] * RAD2DEG,
                                vs068_jangle_max[j],
                                vs068_jangle_min[j],
                            )
                        )

            elif self.errorFlag[i] == -9:
                print("Inverse()を呼び出した後に使用してください。\n")

            else:
                print("お前、何をした・・・・\n")

        print("\n------------------------------\n\n")


if __name__ == "__main__":

    print("VS068 Kinematics")
    print("Version 0.01")
