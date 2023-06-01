# -*- coding: utf-8 -*-
"""
Created on Mon Jan 22 17:03:58 2018

@author: buzz root
"""


import mmap
import struct


def send_flag():
    #　int型を一つ格納可能な共有メモリを検索する
    m1 = mmap.mmap(0, 4, 'saveflag')
    
    # 共有メモリにflagを書き込む
    m1.write(struct.pack("i",1))
    m1.flush()    
    m1.seek(0)
    
    saveflag = int.from_bytes(m1.read(4), 'little')
    m1.seek(0)    
    
    while saveflag!=0:
        saveflag = int.from_bytes(m1.read(4), 'little')
        m1.seek(0)
    
    # 共有メモリのマッピングをやめる
    m1.close()
    
if __name__ == '__main__':

    send_flag()
