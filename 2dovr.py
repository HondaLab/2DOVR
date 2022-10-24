#! /usr/bin/python3
#  Yasushi Honda 2022 10/19

#  各モジュールインポート
import csv
import time
import math
import sys

import numpy as np
#  Pythonファイルインポート 
import modules.motor5a as mt         #  (改良版)モーターを回転させるためのモジュール
import modules.vl53_4a as lidar     #  赤外線レーザーレーダ 3つの場合
import modules.socket as sk

#sokcet 通信関係 
import socket

motor_run = "y"
PERIOD=0.2

GPIO_L = 17     # 左モーターのgpio 17番
GPIO_R = 18     # 右モーターのgpio 18番
MAX_SPEED = 62  # パーセント
DT = 0.1
dt = DT
THRESHOLD = 0.3 # OVMをon/offするための閾値
TURN_TIME=0.3
TURN_POWER=100

# UDP ソケットインスタンス
mt_str_udp=sk.UDP_Recv(sk.robot,sk.motor_port)
vlvr_udp=sk.UDP_Recv(sk.robot,sk.vlvr_port)
area_udp=sk.UDP_Recv(sk.robot,sk.area_port)


# 弾性散乱用の感覚運動写像
def tanh1(x):
    alpha=0.0
    alpha2=1.0
    beta=0.4 # 0.004
    beta2=1000.00
    b=0.4  # 280
    c=0.0
    f=(alpha*math.tanh(beta*(x-b)) + alpha2*math.tanh(beta2*(x-b))+c) / (alpha + alpha2 + c)
    return f

def tanh2(x):
    alpha=0.0
    alpha2=1.0
    beta=0.4 # 0.004
    beta2=1000.00
    b=0.6  # 360
    c=0.0
    f=(alpha*math.tanh(beta*(x-b)) + alpha2*math.tanh(beta2*(x-b))+c) / (alpha + alpha2 + c)
    return f


#  インスタンス生成
mL=mt.Lmotor(GPIO_L)         #  左モーター(gpio17番)
mR=mt.Rmotor(GPIO_R)         #  右モーター(gpio18番)

cnt = 0
data = [0,0,0]
gamma=0.33 # Center weight

areaL=1.0
areaR=1.0

now = time.time()
start = now
init=now
ch='c'
vl=0;vr=0
ovL=0;ovR=0
while ch!='q':
    try:
        data=vlvr_udp.recv()
        cnt+=1
        ovL=data[0]
        ovR=data[1]
    except (BlockingIOError, socket.error):
        pass

    try:
        data=area_udp.recv()
        areaL=data[0]
        areaR=data[1]

        '''
        now=time.time()
        if now-start>PERIOD:
            rate=cnt/(now-start)
            print("\r %6.2f " % (now-init),end="")
            print(" rate=%6.2f " % rate, end="")
            print(" ov_L=%6.2f " % ovL, end="")
            print(" ov_R=%6.2f " % ovR, end="")
            print(" areaL=%6.2f " % areaL, end="")
            print(" areaR=%6.2f " % areaR, end="")
            cnt=0
            start=now
        '''

    except (BlockingIOError, socket.error): # tof recv
        pass

    if motor_run == 'y':
        # 一定時間の擬弾性散乱
        if areaL <= THRESHOLD or areaR <= THRESHOLD:
            if areaL<areaR:
                mL.run(TURN_POWER)
                mR.run(-TURN_POWER)
                time.sleep(TURN_TIME)
            else:
                mL.run(-TURN_POWER)
                mR.run(TURN_POWER)
                time.sleep(TURN_TIME)
        else: # run by OV model
            vl = ovL * MAX_SPEED 
            vr = ovR * MAX_SPEED
            mL.run(vl)
            mR.run(vr)

    try:
       ch=mt_str_udp.recv_str()
    except (BlockingIOError, socket.error):
       pass
    
mR.stop()
mL.stop()
print("#-- #-- #-- #-- #-- #-- #-- #-- #--")
"""
print()
print("===============================")
print("=  実験時間 {:.1f} (sec)".format(now-start))
print("=  q_s--->")
print("===============================")
print()
print("おつかれさまでした  ^-^/")
"""
