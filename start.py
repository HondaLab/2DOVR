#!/usr/bin/python3
import modules.socket as sk
import modules.keyin as keyin
import calc_dist_theta
import ovt  # Optimum Velocity Turn algorithm
from subprocess import Popen
import cv2
import math
import socket
import time
import csv
import datetime
import platform
import numpy as np

PERIOD=0.1 # 'q'を送信する周期 1/rate
SLEEP=0.02
imshow='y'
select_hsv='y'
gamma=0.3333

# 実験パラメータ読み込み
FILE = "parm_ovm.csv" 
def Parameter_read(fp):
    tmp = []
    reader = csv.reader(fp)
    header = next(reader)
    for row in reader:
        if len(row) == 0:
           pass 
        else:
            tmp.append(float(row[0]))
            tmp.append(float(row[1]))
            tmp.append(float(row[2]))
            tmp.append(float(row[3]))
            tmp.append(float(row[4]))
            tmp.append(float(row[5]))
    return tmp
#  パラメータ読み込み
file_pointer = open(FILE,'r')
parm = Parameter_read(file_pointer)

# 最適速度インスタンス
ov = ovt.Optimal_Velocity_class(parm) 

# ------picam --------
str_udp=sk.UDP_Send(sk.robot,sk.cam_port)
cam_udp=sk.UDP_Recv(sk.pc,sk.cam_port)
cmd='ssh pi@'+sk.robot+' 2DOVR/picam.py &'
# 実行後に"&"をつけないと，local(このプログラム)がキーボードを受け付けない．
picam_process=Popen(cmd.strip().split(' '))
# --------------------

# ------tof --------
tof_udp=sk.UDP_Recv(sk.pc,sk.tof_port)
area_udp=sk.UDP_Send(sk.robot,sk.area_port)
tof_str_udp=sk.UDP_Send(sk.robot,sk.tof_str_port)
cmd='ssh pi@'+sk.robot+' 2DOVR/tof.py &'
# 実行後に"&"をつけないと，local(このプログラム)がキーボードを受け付けない．
tof_process=Popen(cmd.strip().split(' '))
left=1;right=1;center=1
LIST_SIZE=10
left_list=[0.0]*LIST_SIZE
center_list=[0.0]*LIST_SIZE
right_list=[0.0]*LIST_SIZE
# --------------------

# ------ 2dovr関連UDPインスタンスとコマンド --------
mt_str_udp=sk.UDP_Send(sk.robot,sk.motor_port)
vlvr_udp=sk.UDP_Send(sk.robot,sk.vlvr_port)
cmd='ssh pi@'+sk.robot+' 2DOVR/2dovr.py &'
# 実行後に"&"をつけないと，local(このプログラム)がキーボードを受け付けない．
#robot_process=Popen(cmd.strip().split(' '))
data=[0.0,0.0,0.0]
# --------------------------------------------------


# ---- thetaの値をファイルに保存するため -----
ex_start_time = datetime.datetime.now()
ex_start_time = str(ex_start_time.strftime('%Y%m%d%H%M%S'))
ex_start_time = ex_start_time.replace("'",'')
ex_start_time = ex_start_time.replace(" ",'')
hostname = '[%s]' % platform.uname()[1]
hostname = hostname.replace("[",'')
hostname = hostname.replace("]",'')

write_file = str(hostname) + "-" +str(ex_start_time) + ".txt"
print(write_file)

write_fp = open("result/"+write_file,"w")
write_fp.write("#"+hostname+"\n")
# ---------------------------------------------

print('Waiting for frame from picam on robot.')
recv=0
while recv==0:
    try: # frameのsizeを決める
        frame=cam_udp.recv_img()
        recv=1
    except (BlockingIOError, socket.error):
        recv=0

width=frame.shape[1]
height=frame.shape[0]
print("# Resolution: %5d x %5d" % (width,height))
size = (width, height)

print('Waiting for tofs fromrobot.')
recv=0
while recv==0:
    try: 
        data=tof_udp.recv()
        recv=1
    except (BlockingIOError, socket.error):
        recv=0

# ----- ロボットカメラからの映像を保存する -----
OUT_FILE="./collected.mp4"
fmt = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
record_fps=30
vw = cv2.VideoWriter(OUT_FILE, fmt, record_fps, size)
# ----------------------------------------------

# 距離distと，角度thetaをframeから計算するためのインスタンス
picam_frame=calc_dist_theta.Picam_frame()
if select_hsv=='y':
    lower,upper=picam_frame.hsv(frame)
else:
    #Red Cup H:S:V=3:140:129
    # h,s,v = 171,106,138
    # 177  139  141 2021/06/01  電気OFF
    # 172  160  148 2021/06/15  電気ON
    # 179  116  101 2021/06/15  電気ON
    # 172  164  152 2021/06/22  電気ON

    H = 172; S = 164; V =152 
    h_range = 20; s_range = 80; v_range = 80 # 明度の許容範囲
    lower = np.array([H-h_range, S-s_range, V-v_range])
    upper = np.array([H+h_range, S+s_range, V+v_range])


key=keyin.Keyboard()
now=time.time()
start=now
init=now
ch='c'
cnt=0
rate=30.0
print("'q'を入力すると終了します．(Input 'q' to quit.)")
while ch!='q':

    try:
        frame=cam_udp.recv_img()
        cnt+=1
        dist,theta=picam_frame.calc(frame,lower,upper)
        if dist == None:
            dist = float(8)
            theta = 0.0
        else:
            # pixyカメラで物体を認識している時
            mode = "picam"
            dist = float(dist)
        last = now
        now = time.time()
        dt = now-last
        vl, vr = ov.calc(dist,theta,dt)
        data[0]=vl
        data[1]=vr
        vlvr_udp.send(data)
        #data.clear()
        
        vw.write(frame)

        if imshow=='y':
            show=cv2.resize(frame,(800,300))
            #ax.imshow(show,extent=[*xlim,*ylim], aspect='auto',alpha=0.6)
            cv2.imshow("frame",show)
            cv2.waitKey(1)
    except (BlockingIOError, socket.error):
        pass

    try:
        data=tof_udp.recv()
        left=data[0]/1000  # 単位をメートルに換算
        right=data[1]/1000
        center=data[2]/1000
        left_av=np.sum(left_list)/LIST_SIZE
        center_av=np.sum(center_list)/LIST_SIZE
        right_av=np.sum(right_list)/LIST_SIZE
        if np.fabs(left-left_av)/100<0.5:
           left_list.pop(0)
           left_list.append(left)
        if np.fabs(center-center_av)/100<0.5:
           center_list.pop(0)
           center_list.append(center)
        if np.fabs(right-right_av)/100<0.5:
           right_list.pop(0)
           right_list.append(right)

        if left>0 and center>0:
            areaL=math.exp(gamma*math.log(center_av))*math.exp((1-gamma)*math.log(left_av))
        if right>0 and center>0:
            areaR=math.exp(gamma*math.log(center_av))*math.exp((1-gamma)*math.log(right_av))

        data[0]=areaL
        data[1]=areaR
        area_udp.send(data)

        '''
        # tanh関数を使った擬弾性散乱
        tof_r = tanh1(areaL)
        tof_l = tanh2(areaR)
        if areaL < THRESHOLD or areaR < THRESHOLD:
            ovL = 1.0
            ovR = 1.0
        vl = ovL * tof_l * MAX_SPEED 
        vr = ovR * tof_r * MAX_SPEED
        '''
       
    except (BlockingIOError, socket.error):
        pass

    if now-start>PERIOD:
        ch=key.read()
        str_udp.send_str(ch) # レイトを落として周期的にchを送信
        mt_str_udp.send_str(ch) 
        tof_str_udp.send_str(ch) 
        rate=cnt/(now-start)
        if dist!=None and theta!=None:
           print("\r time=%5.2f sec" % (now-init), end='')
           print(" rate=%5.2f" % rate, end='')
           print(" dist=%5.2f" % dist, end='')
           print(" theta=%5.2f" % theta, end='')
           #print(" dt=%6.3f" % dt, end='')
           print(" ov_L=%6.2f" % vl, end='')
           print(" ov_R=%6.2f" % vr, end='')
           print(" left_av=%6.2f" % left_av, end='')
           print(" center_av=%6.2f" % center_av, end='')
           print(" right_av=%6.2f" % right_av, end='')
           #print(" areaR=%6.2f" % areaR, end='')
           write_fp.write(str('{:.2g}'.format(now-init))+", ")
           write_fp.write(str(theta) + ", ")
           write_fp.write("\n")
        else:
           print("\r %5.2f %5.2f" % (now-init,rate),end='')

        cnt=0
        start=now


    #time.sleep(SLEEP)

vw.release()        
write_fp.close()

print("bye-bye")

