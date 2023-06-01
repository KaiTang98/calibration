# -*- coding: utf-8 -*-

import glob
import mmap
import os
import struct
import sys
import time
from copy import deepcopy

import cv2
import numpy as np
import pypuclib
from pypuclib import (
    PUC_DATA_MODE,
    Camera,
    CameraFactory,
    Decoder,
    PUCException,
    Resolution,
    XferData,
)

sys.path.append("..")


class utils:
    def __init__(self):
        # 保存フラグ用の共有メモリの作成
        self.m1 = mmap.mmap(0, 4, "saveflag")
        # 終了フラグ用の共有メモリ
        self.m2 = mmap.mmap(0, 4, "endflag")

    def initialize_file(self):
        # -------------------ファイル関連の初期化-------------------
        # 画像フォルダ内のファイルを取得して削除する
        images = glob.glob("./saved_img/*.bmp")
        for i in images:
            os.remove(i)

    def initilize_mmp(self):
        # -------------------フラグ関連の初期化-------------------
        # 保存フラグ用の共有メモリを初期化する
        self.m1.write(struct.pack("i", 0))
        self.m1.flush()
        self.m1.seek(0)

        # 終了フラグ用の共有メモリを初期化する
        self.m2.write(struct.pack("i", 0))
        self.m2.flush()
        self.m2.seek(0)

    def finalize_mmap(self):
        # -------------------フラグ関連の終了処理-------------------
        # 共有メモリを初期化して開放する
        self.m1.write(struct.pack("i", 0))
        self.m1.flush()
        self.m1.close()

        self.m2.write(struct.pack("i", 0))
        self.m2.flush()
        self.m2.close()


class INFINICAM_image_capture(utils):
    def __init__(self):

        super(INFINICAM_image_capture, self).__init__()

        self.mode = 1

        self.pattern_size = (4, 3)  # 交差ポイントの数

        self.x = 0
        self.y = 0
        self.w = 1246
        self.h = 624

    def capture_image(self):
        saveflag = 0
        endflag = 0

        image_num = 0  # 自動保存画像枚数

        cam = CameraFactory().create()
        cam.setFramerateShutter(50, 50)
        decoder = cam.decoder()

        while True:
            xferData = cam.grab()

            # Decode the data can be used as image
            array = decoder.decode(xferData, self.x, self.y, self.w, self.h)
            # array = decoder.decode(xferData)

            img = deepcopy(array)

            # if self.mode == 1:
            #     ret, corner = cv2.findChessboardCorners(array, self.pattern_size)

            # elif self.mode == 2:
            #     ret, corner =  cv2.findCirclesGrid(array, self.pattern_size, None,  cv2.CALIB_CB_SYMMETRIC_GRID+ cv2.CALIB_CB_CLUSTERING)

            # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 100, 0.001)
            # cv2.cornerSubPix(array, corner, (5,5), (-1,-1), criteria)

            # array = cv2.drawChessboardCorners(array, self.pattern_size, corner, ret)

            # フレームを表示する
            cv2.imshow("camera capture", array)

            # 保存フラグ用の共有メモリを読み取る. バイト列で出力されるからintに変換する
            saveflag = int.from_bytes(self.m1.read(4), "little")
            self.m1.seek(0)

            # 終了フラグ用の共有メモリを読み取る. バイト列で出力されるからintに変換する
            endflag = int.from_bytes(self.m2.read(4), "little")
            self.m2.seek(0)

            # 1ms待機する
            k = cv2.waitKey(1)

            # フラグによって保存
            if saveflag == 1:

                # 画像を保存する
                cv2.imwrite("./saved_img/img_%04d.bmp" % image_num, img)

                if (
                    os.path.exists("./saved_img/img_%04d.bmp" % image_num)
                    == True
                ):

                    print("img_%04d.bmp saved!" % image_num)

                    # 保存用フラグを0に戻す
                    self.m1.write(struct.pack("i", 0))
                    self.m1.flush()
                    self.m1.seek(0)

                    image_num += 1

            # sキーで保存を行う
            if k == 115:
                # 画像を保存する
                cv2.imwrite("./saved_img/img_%04d.bmp" % image_num, img)
                image_num += 1
                print("image saved!")

            # フラグによって終了
            if endflag == 1:
                break

            # キーによって終了
            if k == 27:  # ESCキーで終了
                break

        # キャプチャを解放する
        cv2.destroyAllWindows()


def main():
    ut = utils()
    imc = INFINICAM_image_capture()

    ut.initialize_file()
    ut.initilize_mmp()
    imc.capture_image()
    ut.finalize_mmap()

    print("Fin!")


if __name__ == "__main__":
    main()
