#! /usr/bin/python3
#  Yasushi Honda 2021 5/27
#  2dovr_210513.py
#  2021-04-16
#  Masashi Yamada

#  各モジュールインポート
import csv
import time
import math
import sys
import datetime
import platform

import numpy as np
#  Pythonファイルインポート 
import modules.motor5a as mt         #  (改良版)モーターを回転させるためのモジュール
import modules.vl53_4a as lidar     #  赤外線レーザーレーダ 3つの場合
import modules.socket as sk

#sokcet 通信関係 
import socket

motor_run = "y"

GPIO_L = 17     # 左モーターのgpio 17番
GPIO_R = 18     # 右モーターのgpio 18番
MAX_SPEED = 62  # パーセント
DT = 0.1
dt = DT
THRESHOLD = 0.3 # OVMをon/offするための閾値

# UDP ソケットインスタンス
mt_str_udp=sk.UDP_Recv(sk.robot,sk.motor_port)
vlvr_udp=sk.UDP_Recv(sk.robot,sk.vlvr_port)


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

ex_start_time = datetime.datetime.now()
ex_start_time = str(ex_start_time.strftime('%Y%m%d%H%M%S'))
ex_start_time = ex_start_time.replace("'",'')
ex_start_time = ex_start_time.replace(" ",'')

hostname = '[%s]' % platform.uname()[1]
hostname = hostname.replace("[",'')
hostname = hostname.replace("]",'')

write_file = str(hostname) + "-" +str(ex_start_time) + ".txt"
print(write_file)

#write_fp = open("/home/pi/2DOVR/result/"+write_file,"w")
#write_fp.write("#"+hostname+"\n")


#  インスタンス生成
tofL,tofR,tofC=lidar.start() #  赤外線レーザ(3)
#tofL,tofR=lidar.start()       #  赤外線レーザ(2)
print("VL53L0X 接続完了\n")
mL=mt.Lmotor(GPIO_L)         #  左モーター(gpio17番)
mR=mt.Rmotor(GPIO_R)         #  右モーター(gpio18番)

count = 0
data = []
gamma=0.33 # Center weight

now = time.time()
start = now
ch='c'
vl=0;vr=0
while ch!='q':
    try:
        data=vlvr_udp.recv()
        vl=data[0]
        vr=data[1]
    except (BlockingIOError, socket.error):
        pass

    try :
        lidar_distanceL=tofL.get_distance()/1000
        if lidar_distanceL>2:
            lidar_distanceL=2

        lidar_distanceC=tofC.get_distance()/1000
        if lidar_distanceC>2:
            lidar_distanceC=2
           
        lidar_distanceR=tofR.get_distance()/1000
        if lidar_distanceR>2:
            lidar_distanceR=2

        if lidar_distanceL>0 and lidar_distanceC>0:
            areaL=math.exp(gamma*math.log(lidar_distanceC))*math.exp((1-gamma)*math.log(lidar_distanceL))
        if lidar_distanceR>0 and lidar_distanceC>0:
            areaR=math.exp(gamma*math.log(lidar_distanceC))*math.exp((1-gamma)*math.log(lidar_distanceR))

        tof_r = tanh1(areaL)
        tof_l = tanh2(areaR)

        '''
        print("\r %6.2f " % (now-start),end="")
        #print(" dist=%6.2f " % dist, end="")
        #print(" theta=%6.2f " % theta, end="")
        print(" v_L=%6.2f " % vl, end="")
        print(" v_R=%6.2f " % vr, end="")
        print(" dL=%6.2f " % lidar_distanceL, end="")
        print(" dC=%6.2f " % lidar_distanceC, end="")
        print(" dR=%6.2f " % lidar_distanceR, end="")
        #write_fp.write(str('{:.2g}'.format(now-start))+", ")
        #write_fp.write(str(theta) + ", ")
        #write_fp.write("\n")
        '''

        if areaL < THRESHOLD or areaR < THRESHOLD:
            vl = 1.0
            vr = 1.0
        
        vl = vl * tof_l * MAX_SPEED 
        vr = vr * tof_r * MAX_SPEED

        if motor_run == 'y':
            mL.run(vl)
            mR.run(vr)
    except KeyboardInterrupt:
        mR.stop()
        mL.stop()
        write_fp.close()
        #print("Ctrl + C button pressed")
        sys.exit("\nsystem exit ! \n")

    try:
       ch=mt_str_udp.recv_str()
    except (BlockingIOError, socket.error):
       pass
    
mR.stop()
mL.stop()
#write_fp.close()
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
