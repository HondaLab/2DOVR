#!/usr/bin/python3

# Yasushi Honda 2022 6/18

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import socket
import numpy as np
import modules.socket as sk

RES_X=int( 320 )
RES_Y=int( 240 )
view_upper=110
view_lower=190
PERIOD=0.2

cam = PiCamera()
cam.framerate = 30
cam.awb_mode='auto'
cam.iso=800
cam.shutter_speed=1000000
cam.exposure_mode = 'auto' # off, auto, fixedfps
time.sleep(1)
g = cam.awb_gains
cam.awb_mode = 'off'
cam.awb_gains = g
cam.resolution = (RES_X, RES_Y)
cam.rotation=0
cam.meter_mode = 'average' # average, spot, backlit, matrix
cam.exposure_compensation = 0
rawCapture = PiRGBArray(cam, size=(RES_X, RES_Y))


if __name__=='__main__':

   cam_udp=sk.UDP_Send(sk.pc,sk.cam_port)
   recv_udp=sk.UDP_Send(sk.calc,sk.recv_port)
   data=[]
   str_udp=sk.UDP_Recv(sk.robot,sk.cam_port)

   OUT_FILE="collected_img.mp4"
   fmt = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
   record_fps=30
   height=view_lower-view_upper
   #print("# Resolution: %5d x %5d" % (RES_X,height))
   size = (RES_X, height)
   vw = cv2.VideoWriter(OUT_FILE, fmt, record_fps, size)

   now=time.time()
   start=now
   init=now
   cnt=0
   ch='c' 
   while ch!='q':

      cam.capture(rawCapture, format="bgr", use_video_port=True)
      frame = rawCapture.array
      frame2 = frame[view_upper:view_lower,:,:]
      rawCapture.truncate(0)

      vw.write(frame2)

      cam_udp.send_img(frame2)
      recv_udp.send_img(frame2)

      '''
      now=time.time()
      if now-start>PERIOD:
         rate=cnt/PERIOD
         print("\r %8.2f %8.2f %s" % (now-init, rate, ch),end='')
         cnt=0
         start=now
      '''

      cnt+=1
      try:
          ch=str_udp.recv_str()
      except (BlockingIOError,socket.error):
          pass

