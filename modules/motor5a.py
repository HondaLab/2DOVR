#!/usr/bin/python3

# motor5a.py
# Calibrated by tanh function
# 2021 4/17
# Yasushi Honda

import pigpio
import time
import numpy as np

MIN_WIDTH=1000
MID_WIDTH=1500
MAX_WIDTH=2000

class Motor:

   def __init__(self,gpio):
      self.gpio=gpio
      self.pi = pigpio.pi()
      if not self.pi.connected:
         exit()
      self.pi.set_servo_pulsewidth(gpio, MID_WIDTH)

   def move(self,power):
      pwidth=MID_WIDTH+power
      if pwidth>MAX_WIDTH: pwidth=MAX_WIDTH-1
      if pwidth<MIN_WIDTH: pwidth=MIN_WIDTH+1
      self.pi.set_servo_pulsewidth(self.gpio, pwidth)

   def stop(self):
      self.move(0)
      self.pi.stop()

class Lmotor(Motor):
   def run(self,power):
      output=62*np.arctanh(-power/101)+6*np.sign(-power)
      self.move(-power)

class Rmotor(Motor):
   def run(self,power):
      output=62*np.arctanh(power/101)+6*np.sign(power)
      self.move(power)
       
if __name__=='__main__':
   # Motor output
   OUTPUT=61.0

   # Length of time
   Time_Length= 5.0

   # 停止の仕方のメッセージ表示
   print("Robot runs only %5.2f sec." % Time_Length)


   # rate調整用の待ち時間(秒)
   SLEEP=0.0333

   # LeftモーターをGPIO=17に、Rightモーターを18につなぐ。
   motorL = Lmotor(17)
   motorR = Rmotor(18)
   

   # 左右のモーター出力をゼロに初期化する。
   motorL.run(0)
   motorR.run(0)
   time.sleep(1)

   # 各変数の初期化
   left=0
   right=0
   start=time.time()
   now=start
   while now-start<Time_Length:
      left=OUTPUT
      now=time.time() 
      print("\r %5.2f/%5.2f %5.2f" %(now-start,Time_Length,left),end="")

      right=left
      motorL.run(left)
      motorR.run(right)

      # 更新をSLEEP秒だけ待って、rateを調節する。
      time.sleep(SLEEP)


   print("\n")
   # モーター出力をゼロにもどして止める
   motorL.run(0)
   motorR.run(0)
   motorL.stop()
   motorR.stop()
