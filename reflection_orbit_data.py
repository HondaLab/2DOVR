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
import cv2
import datetime
import platform
import numpy as np

# Pythonファイルインポート 
import modules.motor5a as mt     # モーターを回転させるためのモジュール
import modules.vl53_4a as lidar  # 赤外線レーザーレーダ 3つの場合
import modules.keyin as keyin
import file_read as fr

f1 = open('time_distance_data.csv','w',encoding='utf-8')
csv_writer1 = csv.writer(f1) 

def write_data(dt,t,l,c,r,areaL,areaR,pastL,pastR,timer,Ltimer,Rtimer,Ltime,Rtime,Alltime):
    write_data = []
    write_data.append(dt)
    write_data.append(t)
    write_data.append(l)
    write_data.append(c)
    write_data.append(r)
    write_data.append(areaL)
    write_data.append(areaR)
    write_data.append(pastL)
    write_data.append(pastR)
    write_data.append(timer)
    write_data.append(Ltimer)
    write_data.append(Rtimer)
    write_data.append(Ltime)
    write_data.append(Rtime)
    write_data.append(Alltime)
    csv_writer1.writerow(write_data)

motor_run = "y"  # モータを回転させる場合は"y"
show_res = "y"  # モータを回転させる場合は"y"

past_areaR=1.0
past_areaL=1.0
right_timer=0
left_timer=0
start_time=0
stop_time=0
timer=0
adjustment=0.9
write_timer,write_Ltimer,write_Rrimer,write_Ltime_all,write_Rtime_all,write_all=0,0,0,0,0,0


# 弾性散乱のための変数
#TURN_TIME=0.3
TURN_POWER=90

SLEEP = 0.05
BUS = 1         # bus number
I2C_ADDR = 0x54 # I2Cアドレス
GPIO_L = 17     # 左モーターのgpio 17番
GPIO_R = 18     # 右モーターのgpio 18番
MAX_SPEED = 62  # パーセント
DT = 0.05
dt = DT
THRESHOLD = 0.3 # OVMをon/offするための閾値
EX_TIME = 5.3

def motor_out_adjust(vl,vr):
    if vl > 100:
        vl = 100
    if vl < -100:
        vl = -100

    if vr > 100:
        vr = 100
    if vr < -100:
        vr = -100

    return vl,vr

def tof_adjust(distL,distC,distR):
    if distL > 2:
        distL = 2

    if distC > 2:
        distC = 2

    if distR > 2:
        distR = 2

    return distL,distC,distR

#  インスタンス生成
tofL,tofR,tofC=lidar.start() #  赤外線レーザ(3)
print("VL53L0X 接続完了\n")
time.sleep(2)
mL=mt.Lmotor(GPIO_L)         #  左モーター(gpio17番)
mR=mt.Rmotor(GPIO_R)         #  右モーター(gpio18番)
key = keyin.Keyboard()

data = []
gamma=0.33 # Center weight

print("#-- #-- #-- #-- #-- #-- #-- #-- #--")

start = time.time()
now = start
last=now

vl=0;vr=0
ch = key.read()
while ch!="q":
    #  実験中
    try :
        distanceL=tofL.get_distance()/1000

        distanceC=tofC.get_distance()/1000
           
        distanceR=tofR.get_distance()/1000

        distanceL,distanceC,distanceR = tof_adjust(distanceL,distanceC,distanceR)

        if distanceL>0 and distanceC>0:
            areaL=math.exp(gamma*math.log(distanceC))*math.exp((1-gamma)*math.log(distanceL))
        if distanceR>0 and distanceC>0:
            areaR=math.exp(gamma*math.log(distanceC))*math.exp((1-gamma)*math.log(distanceR))

        # vl,vrは2次元最適速度モデルで決定される速度
        

        #else:
            #vl,vr = motor_out_adjust(MAX_SPEED,MAX_SPEED)
        vl,vr = motor_out_adjust(MAX_SPEED,MAX_SPEED)

        if areaL >= THRESHOLD and areaR >= THRESHOLD:
            if past_areaL < THRESHOLD or past_areaR < THRESHOLD:
                if past_areaL<past_areaR:
                    mL.run(TURN_POWER)
                    mR.run(-TURN_POWER)
                    right_timer = right_timer + timer
                    #time.sleep(TURN_TIME)
                    time.sleep(adjustment*right_timer)
                    write_Rtime_all = right_timer + adjustment * right_timer
                    write_all = right_timer + adjustment * right_timer
                    right_timer=0
                else:
                    mL.run(-TURN_POWER)
                    mR.run(TURN_POWER)
                    left_timer = left_timer + timer
                    #time.sleep(TURN_TIME)
                    time.sleep(adjustment*left_timer)
                    write_Ltime_all = left_timer + adjustment * left_timer
                    write_all = left_timer + adjustment * left_timer
                    left_timer=0
            else:
                mL.run(vl)
                mR.run(vr)
        else:
        #if areaL < THRESHOLD or areaR < THRESHOLD:
            if areaL<areaR:
                if past_areaL < THRESHOLD or past_areaR < THRESHOLD:
                    stop_time = time.time()
                    timer = start_time - stop_time
                mL.run(TURN_POWER)
                mR.run(-TURN_POWER)
                start_time = time.time()
                right_timer = right_timer + timer
                write_timer = timer
                write_Rtimer = timer
                timer = 0
                #time.sleep(TURN_TIME)
            else:
                if past_areaL < THRESHOLD or past_areaR < THRESHOLD:
                    stop_time = time.time()
                    timer = start_time - stop_time
                mL.run(-TURN_POWER)
                mR.run(TURN_POWER)
                start_time = time.time()
                left_timer = left_timer + timer
                write_timer = timer
                wright_Ltimer = timer
                timer = 0
                #time.sleep(TURN_TIME)

        past_areaL=areaL
        past_areaR=areaR
        
        if show_res == 'y':
            print("\r %6.2f " % (now-start),end="")
            #print(" dist=%6.2f " % dist, end="")
            #print(" theta=%6.2f " % theta, end="")
            #print(" v_L=%6.2f " % vl, end="")
            #print(" v_R=%6.2f " % vr, end="")
            print(" dL=%6.2f " % distanceL, end="")
            print(" dC=%6.2f " % distanceC, end="")
            print(" dR=%6.2f " % distanceR, end="")
            print(" areaL=%6.2f " % areaL, end="")
            print(" areaR=%6.2f " % areaR, end="")

        #if motor_run == 'y':
            #mL.run(vl)
            #mR.run(vr)
        

        time.sleep(DT)
        last = now
        now = time.time()
        dt = now-last
        ch = key.read()
        wt = time.time()
        write_data(dt,wt,distanceL,distanceC,distanceR,areaL,areaR,past_areaL,past_areaR,write_timer,write_Ltimer,write_Rrimer,write_Ltime_all,write_Rtime_all,write_all)
        write_timer,write_Ltimer,write_Rrimer,write_Ltime_all,write_Rtime_all,write_all=0,0,0,0,0,0
    except KeyboardInterrupt:
        mR.stop()
        mL.stop()
        sys.exit("\nsystem exit ! \n")
mR.stop()
mL.stop()
print("#-- #-- #-- #-- #-- #-- #-- #-- #--")
