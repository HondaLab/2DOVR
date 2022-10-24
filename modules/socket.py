import socket
import numpy
import cv2

# IPアドレス
robot='172.16.8.100'  # ロボット
pc = '172.16.8.181'   # 操作端末PC
calc = '172.16.8.181' # 計算サーバ

# １つの計算サーバで recv_data.py / calc_nn_h1.py を
# 複数動かす場合は，以下のポート番号の重複を避ける.
vlvr_port = 50001
tof_port = 50002
tof_str_port = 50003
motor_port = 50004
cam_port = 50005
svd_port = 50006
recv_port = 50007
area_port = 50008

class UDP_Send():
    def __init__(self,addr,port):
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.addr = addr
        self.port = port

    def send_str(self,str):
        self.sock.sendto(str.encode('utf-8'),(self.addr,self.port))
        return 0

    def send(self,lis):
        strig = ''
        num = len(lis)
        i = 0
        while i<num:
            strig = strig + str("%12.8f"%lis[i])
            if i != num-1:
                strig = strig+','
            i = i+1
        self.sock.sendto(strig.encode('utf-8'),(self.addr,self.port))
        return 0

    def send_img(self,frame):
        jpegstring=cv2.imencode('.jpg',frame)[1].tostring()
        self.sock.sendto(jpegstring,(self.addr,self.port))
        return 0
            
		
class UDP_Recv():
    def __init__(self,addr,port):
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind((addr,port))
        self.sock.setblocking(0)

    def recv_str(self):
        message = self.sock.recv(15260).decode('utf-8')
        return message

    def recv(self):
        message = self.sock.recv(15260).decode('utf-8')
        slist = message.split(',')
        a = [float(s) for s in slist]
        return a

    def recv_img(self):
        data = self.sock.recv(1048576)
        narray = numpy.fromstring(data,dtype='uint8')
        frame=cv2.imdecode(narray,1)
        return frame
                
