#!/usr/bin/python3

# 2次元最適速度モデル -> 最適速度旋回アルゴリズム(ovt)

import math

# Optimal Velocity Turn
class Optimal_Velocity_class:
    def __init__(self,parm):
        self.vs = parm[0]    #相互作用なしで動き続けるための項 
        self.a = parm[1]     #感応度
        self.alpha = parm[2] #最高速度決定
        self.beta = parm[3]  #αβで最適速度関数の変化率を決定
        self.b = parm[4]     #変曲点のx座標(ロボットの車頭距離にする)
        self.c = parm[5]     #前進後退の割合決定
        self.d = 2.00     # ロボット半径
        self.v = 0.0
        self.omega = 0.0
        print("#     a ",end="")
        print("  alpha ",end="")
        print("   beta ",end="")
        print("      b ",end="")
        print("      c ",end="")
        print("      d ")
        print("%7.3f " % self.a,end="")
        print("%7.3f " % self.alpha,end="")
        print("%7.3f " % self.beta,end="")
        print("%7.3f " % self.b,end="")
        print("%7.3f " % self.c,end="")
        print("%7.3f " % self.d)
   
    def calc(self,distance,theta,dt):

        f = self.alpha*(math.tanh(self.beta*(distance - self.b)) + self.c ) #(3)式
        ov = (1.0*math.cos(theta))*f
        a = self.a*(self.vs + ov - self.v) 
        a_omega = self.a*(theta-self.omega) 
 
        self.v = self.v + a*dt
        self.omega = self.omega + a_omega*dt
    
        left = self.v + self.d * self.omega
        right = self.v - self.d * self.omega

        left = left/(self.alpha*(1+math.fabs(self.c)))
        right = right/(self.alpha*(1+math.fabs(self.c)))
        
        return left,right
