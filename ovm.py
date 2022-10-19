#!/usr/bin/python3
"""
作成日：2020/10/31
更新日：2021/04/16
作成者：20043060 山田 将司
###説明###
ssr系ロボット用2次元最適速度モデル
ovm.pyに関しては特に改造する必要無し．
"""
#モジュールインポート
import math

# 定数定義
MAX = 3 #自身を除いたロボット最大数

# 平方根計算，math.と書きたくないため，自作
def sqrt_1(x):
    return math.sqrt(x)
def sqrt_2(x,y):
    return math.sqrt(x*x + y*y)

# 絶対値計算、math.と書きたくないため，自作
def fabs(x):
    return math.fabs(x)

# sin計算，math.と書きたくないため，自作
def sin(x):
    return math.sin(x)

# cos計算，math.と書きたくないため，自作
def cos(x):
    return math.cos(x)

# arc-cos計算，math.と書きたくないため，自作
def acos(x):
    try:
        return math.acos(x)
    except:
        return 0.0

# hyperbolic-tan計算，math.と書きたくないため，自作
def tanh(x):
    return math.tanh(x)

# シグナム関数(符号関数)
def sgn(x):
    if x > 0:
        sign = 1
    if x == 0:
        sign = 0
    if x < 0:
        sign = -1
    return sign

# Optimal Velocity model
class Optimal_Velocity_class:
    def __init__(self,parm):
        self.vs = parm[0]    #相互作用なしで動き続けるための項 
        self.a = parm[1]     #感応度
        self.alpha = parm[2] #最高速度決定
        self.beta = parm[3]  #αβで最適速度関数の変化率を決定
        self.b = parm[4]     #変曲点のx座標(ロボットの車頭距離にする)
        self.c = parm[5]     #前進後退の割合決定
        self.vx = 0.0
        self.vy = 0.1
        self.d = 5.00
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

        f_rkj = self.alpha*(tanh(self.beta*(distance - self.b)) + self.c ) #(3)式
        
        #print("\r f_rkj=%7.3f" % f_rkj,end="")

        nx = sin(theta)
        ny = cos(theta)

        large_v_x = (1+cos(theta)) * f_rkj * nx
        large_v_y = (1+cos(theta)) * f_rkj * ny

        ax = self.a * (large_v_x - self.vx)
        ay = self.a * (large_v_y - self.vy)

        self.vx = self.vx + dt * ax
        self.vy = self.vy + dt * ay

        omega=math.atan(self.vx/self.vy)

        left = self.vy + self.d * omega
        right = self.vy - self.d * omega

        left = left/(2.0*self.alpha*(1+math.fabs(self.c)))
        right = right/(2.0*self.alpha*(1+math.fabs(self.c)))
        
        return left,right,omega
