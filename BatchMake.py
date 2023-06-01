# -*- coding: utf-8 -*-
import codecs
import csv
import os

import numpy as np

if __name__ == "__main__":

    # os.chdir("Batch_Make")

    time1 = 10
    time2 = 2

    file_name = "angle_100"
    file_name2 = "angle_100"

    # str_des_angle = '-36.88 24.47 50.87 -79.25 54.49 20.4'
    # str_des_angle = "38.55 25.87 47.36 77.0 53.21 -13.85"

    # 保存してある関節角を読み込む
    robot_name = "B"
    cur_angle_deg = np.loadtxt(
        "./data/" + robot_name + "/current_angle.csv"
    ).astype(np.float32)
    cur_angle_deg_str = cur_angle_deg.astype(str)
    str_des_angle = " ".join(cur_angle_deg_str)

    # -----------------x_trainの取得-----------------
    angle = np.empty((0, 6), dtype=np.float32)
    with codecs.open(
        "data/"  + robot_name + "/"+ file_name + ".csv", "r", "utf-8-sig"
    ) as f:  # ここが間違ってないか要確認
        angle_reader = csv.reader(f, delimiter=",")
        for row in angle_reader:
            row = np.expand_dims(row, axis=0)
            angle = np.append(angle, row, axis=0)
        f.close()

    angle = angle.astype("float32")

    os.chdir("../../../Batch/Kai/Calibration")

    f = open("Calib_" + file_name2 + ".txt", "w")

    f.write(",Get_Calib_Img,LABEL,,.exe\n")
    f.write(",ServoON,ServoON,0,.exe\n")
    f.write(",CameraON,StartCamera_Calib,NOWAIT,.exe\n")

    # 初期位置から目標位置に移動する
    f.write("●,StartPos,FifthPolynomialRT,")
    f.write(str_des_angle + " ")  # 目標位置を変更したらここを変更すること
    f.write(str(time1) + ",.exe\n")

    for l in range(0, np.size(angle, axis=0)):

        # 手先位置を微小移動させる
        f.write(",Move,FifthPolynomialRT,")

        for k in range(0, 6):
            f.write("{0:f}".format(angle[l][k]) + " ")

        f.write(str(time2) + ",.exe\n")
        f.write(",SaveImg,SaveImg_Calib,0,.exe\n")

    f.write("●,Back2StartPos,FifthPolynomialRT,")
    f.write(str_des_angle + " ")  # 目標位置を変更したらここを変更すること
    f.write(str(time2) + ",.exe\n")

    f.write(",ServoOFF,ServoOFF,,.exe\n")
    f.write(",CameraOFF,EndCamera_Calib,NOWAIT,.exe")
    f.close()

    print("OK\n")
