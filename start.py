#!/usr/bin/python3
import modules.socket as sk
import modules.keyin as keyin
import calc_dist_theta
from subprocess import Popen
import cv2
import socket
import time

PERIOD=0.1 # 'q'を送信する周期 1/rate
SLEEP=0.02
imshow='y'
select_hsv='y'

# ------picam --------
str_udp=sk.UDP_Send(sk.robot,sk.cam_port)
cam_udp=sk.UDP_Recv(sk.pc,sk.cam_port)
cmd='ssh pi@'+sk.robot+' 2DOVR/picam.py &'
# 実行後に"&"をつけないと，local(このプログラム)がキーボードを受け付けない．
picam_process=Popen(cmd.strip().split(' '))
# --------------------

# ------ 2dovr --------
mt_str_udp=sk.UDP_Send(sk.robot,sk.motor_port)
dist_udp=sk.UDP_Send(sk.robot,sk.dist_port)
cmd='ssh pi@'+sk.robot+' 2DOVR/2dovr.py &'
# 実行後に"&"をつけないと，local(このプログラム)がキーボードを受け付けない．
#robot_process=Popen(cmd.strip().split(' '))
data=[]
# --------------------------

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
print("データを受信するには，recv_data.pyを実行してください．")
print("'q'を入力すると終了します．(Input 'q' to quit.)")
while ch!='q':

    try:
        frame=cam_udp.recv_img()
        dist,theta=picam_frame.calc(frame,lower,upper)
        data.append(dist)
        data.append(theta)
        #dist_udp.send(data)
        data.clear()
        
        vw.write(frame)
        cnt+=1
        if imshow=='y':
            show=cv2.resize(frame,(800,480))
            #ax.imshow(show,extent=[*xlim,*ylim], aspect='auto',alpha=0.6)
            cv2.imshow("frame",show)
            cv2.waitKey(1)
    except (BlockingIOError, socket.error):
        pass

    now=time.time()
    if now-start>PERIOD:
        ch=key.read()
        str_udp.send_str(ch) # レイトを落として周期的にchを送信
        mt_str_udp.send_str(ch) 
        rate=cnt/(now-start)
        if dist!=None and theta!=None:
           print("\r %5.2f %5.2f %5.2f %5.2f" % (now-init,rate,dist,theta),end='')
        else:
           print("\r %5.2f %5.2f" % (now-init,rate),end='')

        cnt=0
        start=now

    #time.sleep(SLEEP)

vw.release()        

print("bye-bye")

