# -*- coding: utf-8 -*-
"""
Created on Mon Aug 30 20:32:33 2021

@author: root
"""

import numpy as np

###############################################################################################
# ##キャリブレーション用パラメータ###############################################################
###############################################################################################
# チェッカーパターンパラメータ
SQUARE_SIZE = 1.5
PATTERN_SIZE = (4, 3)
###############################################################################################
###############################################################################################


###############################################################################################
# ##ロボット用パラメータ###############################################################
###############################################################################################
# 関節角のリミット 適宜緩めてください。
ANGLE_MAX_DEG = [165.0, 130.0, 148.0, 265.0, 115.0, 360.0]
ANGLE_MIN_DEG = [-165.0, -95.0, -115.0, -265.0, -115.0, -360.0]

###############################################################################################
###############################################################################################


###############################################################################################
# ##カメラ用パラメータ###############################################################
###############################################################################################
# #元データの大きさ
WIDTH = 1246
# HEIGHT = 1008
HEIGHT = 624

# ORIGIN_WIDTH = 1246
# ORIGIN_HEIGHT = 1008
# #デコード後の大きさ
DECODED_W = 720
DECODED_H = 540

# #開始点
EXT_W = 240
EXT_H = 240

# fps
FPS = 250

# #露光時間 = 1/exp_time
EXP_TIME = 250

FLAG_SIZE = 10
##############################################################################
###############################################################################################


###############################################################################################
# ##PCループ#####################################################################
###############################################################################################
HZ = 4000

###############################################################################################
# #画像処理用パラメータ############################################################
###############################################################################################

HAND_VELOCITY = 1.0
# 薄地1mm
# 普通地2.5mm
# 厚地3mm
R = 0.3  # cm
STITCH = 3.0  # mm

# #輝度値の閾値
ROI_BRIGHT = 200

# THRESHOLD = 150
THRESHOLD = 80

CANNY_MIN = 180
CANNY_MAX = 240

# GAMMA = 1.5
# GAMMA = 0.6
GAMMA = 0.7

VIDEO = 60

ROI_BRIGHT = 200
MASK_BRIGHT = 255 - ROI_BRIGHT

# #針の位置(画像上)
NEEDLE_PX_X = 636
NEEDLE_PX_Y = 495
# 紙用
# NEEDLE_PX_Y = 500

NEEDLE_X = NEEDLE_PX_X - EXT_W
ROI_OFFSET_X_MIN = 200
ROI_OFFSET_X_MAX = 0
ROI_LENGTH_X = ROI_OFFSET_X_MIN + ROI_OFFSET_X_MAX  # 8の倍数になるように
# ROI_LENGTH_X = 120

DECODE_ORIGIN_X = NEEDLE_PX_X // 8 * 8 - ROI_OFFSET_X_MIN
DECODE_OFFSET_X = NEEDLE_PX_X % 8
# 布の時の針の位置
# NEEDLE_Y = 495 - EXT_H
NEEDLE_Y = NEEDLE_PX_Y - EXT_H
ROI_OFFSET_Y_MIN = 200
ROI_OFFSET_Y_MAX = 200
ROI_LENGTH_Y = ROI_OFFSET_Y_MIN + ROI_OFFSET_Y_MAX  # 8の倍数になるように
# ROI_LENGTH_Y = 120


DECODE_ORIGIN_Y = NEEDLE_PX_Y // 8 * 8 - ROI_OFFSET_Y_MIN
DECODE_OFFSET_Y = NEEDLE_PX_Y % 8

ROI_MIN_X = NEEDLE_X - 200
# ROI_MIN_X = 0
ROI_MAX_X = NEEDLE_X

ROI_MIN_Y = NEEDLE_Y - 200
# ROI_MIN_Y = 0
ROI_MAX_Y = NEEDLE_Y + 200

###############################################################################################
###############################################################################################


def main():
    import os
    import shutil
    from pathlib import Path

    cur_file = __file__
    parent_path = Path(__file__).parents[1]
    path, file_name = os.path.split(cur_file)
    dir_name = []
    dir_name.append("PositionBase")
    dir_name.append("Calibration")
    dir_name.append("Cloth_PoseEstimation")
    for i in dir_name:
        _temp_dir = os.path.join(parent_path, i)
        _temp_file = os.path.join(_temp_dir, file_name)
        if not _temp_file == cur_file:
            if os.path.exists(_temp_file):
                os.remove(_temp_file)
            original = cur_file
            copy2 = _temp_file
            shutil.copy2(original, copy2)


if __name__ == "__main__":
    main()
