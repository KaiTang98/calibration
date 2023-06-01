# -*- coding: utf-8 -*-
"""
Created on Mon Jan 22 17:03:58 2018

@author: buzz root
"""


import mmap
import struct

def send_flag():
    #　int型を一つ格納可能な共有メモリを検索する
    m2 = mmap.mmap(0, 4, 'endflag')
    
    # 共有メモリにflagを書き込む
    m2.write(struct.pack("i",1))
    m2.flush()
    m2.seek(0)

    # 終了する
    m2.close()    

    
if __name__ == '__main__':

    send_flag()
