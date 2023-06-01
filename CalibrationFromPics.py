# -*- coding: utf-8 -*-

import codecs
import csv
import math

import cv2
import numpy as np

from CalcBetweenRobot import CalcBetweenRobot
from utils import VS068_Kinematics as K


class load_data:
    def __init__(self):

        self.debugmode = 0  # 0: internal parameters are calculated at the same time, 1: pre-computed internal parameters are loaded

        self.reference_img = 120  # Number of reference images
        self.square_size = 1.2  # Size of one side of a square [cm].
        self.pattern_size = (3, 4)  # Number of crossing points
        self.mode = 1

        self.pattern_points = np.zeros(
            (np.prod(self.pattern_size), 3), np.float32
        )
        self.pattern_points[:, :2] = np.indices(self.pattern_size).T.reshape(
            -1, 2
        )
        self.pattern_points *= self.square_size
        # print(self.pattern_points)

    def get_angle(self, robot):

        # Variable for storing acquired joint angles
        all_angle = np.empty((0, 6), dtype=np.float32)

        # Loading of fingertip position posture
        file_name = "angle_" + str(self.reference_img)
        # file_name = 'angle_100'

        with codecs.open(
            "./data/" + robot + "/" + file_name + ".csv", "r", "utf-8-sig"
        ) as f:
            angle_reader = csv.reader(f, delimiter=",")
            for row in angle_reader:
                row = np.expand_dims(row, axis=0)
                all_angle = np.append(all_angle, row, axis=0)
            f.close()

        return all_angle

    def get_data_for_calib(self, all_angle, robot):

        objpoints = []
        imgpoints = []
        good_angle = []

        for img_num in range(self.reference_img):

            img = cv2.imread(
                "./saved_img/" + robot + "/img_%04d.bmp" % img_num
            )

            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

            if self.mode == 1:
                # Detecting chessboard corners
                ret, corner = cv2.findChessboardCorners(
                    gray, self.pattern_size
                )

            if self.mode == 2:
                ret, corner = cv2.findCirclesGrid(
                    gray,
                    self.pattern_size,
                    None,
                    cv2.CALIB_CB_CLUSTERING | cv2.CALIB_CB_SYMMETRIC_GRID,
                )

            # If there is a corner
            if ret == True:
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT,
                    100,
                    0.001,
                )
                cv2.cornerSubPix(gray, corner, (5, 5), (-1, -1), criteria)

                print("img_%04d.bmp: seccess" % img_num)
                good_angle.append(all_angle[img_num])
                imgpoints.append(corner.reshape(-1, 2))
                objpoints.append(self.pattern_points)

                # Corner Drawing
                gray = cv2.drawChessboardCorners(
                    gray, self.pattern_size, corner, ret
                )

            else:
                print("img_%04d.bmp: fail" % img_num)

        return good_angle, imgpoints, objpoints, gray


class calibrate_intrinsic(load_data):
    def __init__(self):

        super(calibrate_intrinsic, self).__init__()

    def get_intrinsic(self, good_angle, imgpoints, objpoints, gray, robot):

        if self.debugmode == 0:

            (
                ret,
                mtx,
                dist,
                Rvec_target2cam,
                tvec_target2cam,
            ) = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )

            np.save("data/" + robot + "/mtx", mtx)
            np.save("data/" + robot + "/dist", dist)

            tot_error = 0
            for obj_point_num in range(len(objpoints)):
                imgpoints_reproj, _ = cv2.projectPoints(
                    objpoints[obj_point_num],
                    Rvec_target2cam[obj_point_num],
                    tvec_target2cam[obj_point_num],
                    mtx,
                    dist,
                )

                error = 0
                for img_point_num in range(imgpoints_reproj.shape[0]):
                    error += np.linalg.norm(
                        imgpoints[obj_point_num][img_point_num]
                        - imgpoints_reproj[img_point_num]
                    )

                tot_error += error / len(imgpoints_reproj)

            print(
                "average of error (intrinsic): "
                + str(tot_error / len(objpoints))
                + " pixel"
            )

            T_cam2target = np.zeros((len(Rvec_target2cam), 4, 4), np.float32)
            T_target2cam = np.zeros((len(Rvec_target2cam), 4, 4), np.float32)

            for i in range(len(T_cam2target)):
                R_target2cam, jac = cv2.Rodrigues(Rvec_target2cam[i])
                T_target2cam[i][0:3, 0:3] = R_target2cam
                T_target2cam[i][0:3, 3:4] = tvec_target2cam[i][0:3]
                T_target2cam[i][3][3] = 1
                T_cam2target[i] = np.linalg.inv(T_target2cam[i])

            np.save("data/" + robot + "/T_cam2target", T_cam2target)
            np.save("data/" + robot + "/T_target2cam", T_target2cam)

        if self.debugmode == 1:

            Rvec_target2cam = []
            tvec_target2cam = []
            mtx = np.load("data/" + robot + "/mtx.npy")
            dist = np.load("data/" + robot + "/dist.npy")

            mtx = np.load("data/A/mtx.npy")
            dist = np.load("data/A/dist.npy")

            for i in range(len(objpoints)):
                err, R, t = cv2.solvePnP(objpoints[i], imgpoints[i], mtx, dist)
                Rvec_target2cam.append(R)
                tvec_target2cam.append(t)

            T_cam2target = np.zeros((len(Rvec_target2cam), 4, 4), np.float32)
            T_target2cam = np.zeros((len(Rvec_target2cam), 4, 4), np.float32)

            for i in range(len(T_cam2target)):
                R_target2cam, jac = cv2.Rodrigues(Rvec_target2cam[i])
                T_target2cam[i][0:3, 0:3] = R_target2cam
                T_target2cam[i][0:3, 3:4] = tvec_target2cam[i][0:3]
                T_target2cam[i][3][3] = 1
                T_cam2target[i] = np.linalg.inv(T_target2cam[i])

            np.save("data/" + robot + "/T_cam2target", T_cam2target)
            np.save("data/" + robot + "/T_target2cam", T_target2cam)

        else:

            T_cam2target = np.load("data/" + robot + "/T_cam2target.npy")
            T_target2cam = np.load("data/" + robot + "/T_target2cam.npy")

        print("intrinsic saved!")

        return T_cam2target, T_target2cam


class calibrate_extrinsic:
    def __init__(self):
        super(calibrate_extrinsic, self).__init__()

        self.DEG2RAD = math.pi / 180.0
        self.RAD2DEG = 180.0 / math.pi

        self.kine = K.KINE()

        # extension specification
        # self.kine.Extension.x = -138.0
        # self.kine.Extension.z = 110.0

    def get_Gripper2Base(self, good_angle, robot):

        T_base2gripper = np.zeros((len(good_angle), 4, 4), np.float32)
        T_gripper2base = np.zeros((len(good_angle), 4, 4), np.float32)

        J = np.zeros((4, 4), dtype=np.float32)

        good_angle = np.asarray(good_angle, np.float32)

        for i in range(len(good_angle)):
            self.kine.Angle[0] = good_angle[i][0] * self.DEG2RAD
            self.kine.Angle[1] = good_angle[i][1] * self.DEG2RAD
            self.kine.Angle[2] = good_angle[i][2] * self.DEG2RAD
            self.kine.Angle[3] = good_angle[i][3] * self.DEG2RAD
            self.kine.Angle[4] = good_angle[i][4] * self.DEG2RAD
            self.kine.Angle[5] = good_angle[i][5] * self.DEG2RAD

            self.kine.Forward()

            J = self.kine.T

            # cmに直す
            J[0][3] = 0.1 * J[0][3]
            J[1][3] = 0.1 * J[1][3]
            J[2][3] = 0.1 * J[2][3]

            T_gripper2base[i] = J
            T_base2gripper[i] = np.linalg.inv(J)

        # Difference between the maximum and minimum values of x,y,z
        print(
            "diff of xyz (cm)):",
            max([r[0] for r in T_gripper2base[:, 0:3, 3:4]])
            - min([r[0] for r in T_gripper2base[:, 0:3, 3:4]]),
            max([r[1] for r in T_gripper2base[:, 0:3, 3:4]])
            - min([r[1] for r in T_gripper2base[:, 0:3, 3:4]]),
            max([r[2] for r in T_gripper2base[:, 0:3, 3:4]])
            - min([r[2] for r in T_gripper2base[:, 0:3, 3:4]]),
        )

        np.save("data/" + robot + "/T_base2gripper", T_base2gripper)
        np.save("data/" + robot + "/T_gripper2base", T_gripper2base)

        print("gripper2base saved!")

        return T_base2gripper, T_gripper2base

    def handeye_calib(
        self, T_base2gripper, T_gripper2base, T_cam2target, T_target2cam, robot
    ):

        T_cam2base = np.zeros((4, 4), np.float32)

        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            T_base2gripper[:, 0:3, 0:3],
            T_base2gripper[:, 0:3, 3:4],
            T_target2cam[:, 0:3, 0:3],
            T_target2cam[:, 0:3, 3:4],
            cv2.CALIB_HAND_EYE_TSAI,
        )

        T_cam2base[0:3, 0:3] = R_cam2base[:]
        T_cam2base[0:3, 3:4] = t_cam2base[:]
        T_cam2base[3, 3] = 1

        print("cam2base \n", T_cam2base)

        np.save("data/" + robot + "/T_cam2base", T_cam2base)
        np.save("data/" + robot + "/T_base2cam", np.linalg.inv(T_cam2base))

        # print("cam2base saved!")

    def robot_world_handeye_calib(
        self, T_base2gripper, T_gripper2base, T_cam2target, T_target2cam, robot
    ):

        T_base2cam = np.zeros((4, 4), np.float32)
        T_gripper2target = np.zeros((4, 4), np.float32)

        (
            R_base2cam,
            t_base2cam,
            R_gripper2target,
            t_gripper2target,
        ) = cv2.calibrateRobotWorldHandEye(
            T_cam2target[:, 0:3, 0:3],
            T_cam2target[:, 0:3, 3:4],
            T_base2gripper[:, 0:3, 0:3],
            T_base2gripper[:, 0:3, 3:4],
            cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
        )

        T_base2cam = self.vec2T(R_base2cam, t_base2cam)
        T_gripper2target = self.vec2T(R_gripper2target, t_gripper2target)

        T_cam2base = np.linalg.inv(T_base2cam)
        T_target2gripper = np.linalg.inv(T_gripper2target)

        print("cam2base: \n", T_cam2base)

        np.save("data/" + robot + "/T_gripper2target", T_gripper2target)
        np.save("data/" + robot + "/T_target2gripper", T_target2gripper)

        np.save("data/" + robot + "/T_cam2base", T_cam2base)
        np.save("data/" + robot + "/T_base2cam", T_base2cam)

    def vec2T(self, R, t):
        T = np.zeros((4, 4), np.float32)
        T[0:3, 0:3] = R[:]
        T[0:3, 3:4] = t[:]
        T[3, 3] = 1

        return T


def main():
    # Camera Robot Calibration
    # ld = load_data()
    # i_calib = calibrate_intrinsic()
    # e_calib = calibrate_extrinsic()

    # all_angle_A = ld.get_angle("A")
    # all_angle_B = ld.get_angle("B")

    # good_angle_A, imgpoints_A, objpoints_A, gray_A = ld.get_data_for_calib(
    #     all_angle_A, "A"
    # )
    # good_angle_B, imgpoints_B, objpoints_B, gray_B = ld.get_data_for_calib(
    #     all_angle_B, "B"
    # )

    # c2t_A, t2c_A = i_calib.get_intrinsic(
    #     good_angle_A, imgpoints_A, objpoints_A, gray_A, "A"
    # )
    # c2t_B, t2c_B = i_calib.get_intrinsic(
    #     good_angle_B, imgpoints_B, objpoints_B, gray_B, "B"
    # )

    # b2g_A, g2b_A = e_calib.get_Gripper2Base(good_angle_A, "A")
    # b2g_B, g2b_B = e_calib.get_Gripper2Base(good_angle_B, "B")

    # e_calib.handeye_calib(b2g_A, g2b_A, c2t_A, t2c_A, "A")
    # e_calib.handeye_calib(b2g_B, g2b_B, c2t_B, t2c_B, "B")

    # e_calib.robot_world_handeye_calib(b2g_A, g2b_A, c2t_A, t2c_A, "A")
    # e_calib.robot_world_handeye_calib(b2g_B, g2b_B, c2t_B, t2c_B, "B")

    # Robot-to-Robot Calibration
    calc = CalcBetweenRobot()

    T_B2A = calc.get_T_B2A()
    T_A2World = calc.get_T_A2World(T_B2A)
    T_B2World = calc.get_T_B2World(T_B2A, T_A2World)
    calc.save_T(T_B2World, T_A2World)

    print(T_B2World)
    print(T_A2World)


if __name__ == "__main__":
    main()
