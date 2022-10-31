
Two dimensional optimal velocity robot
====

## Overview
2次元最適速度ロボット(2dOV Robot)とは，2dOVモデルに基いてその行動（モーターの駆動）が生成される
走行ロボットのことです．

2dOVモデルとは２次元平面における生物などの行動を数理モデル化したものです．
紐状の運動など，変化に富んだ集団行動を再現することが知られています．
このレポジトリでは，その2dOVモデルをアルゴリズム化し，２次元平面を走行するロボットに適用するプログラムを開発します．

## Description
詳しくは
[このレポジトリのwiki](https://github.com/HondaLab/2DOVR/wiki)を参照してください．

## Requirement
  * Raspberry Pi3
  * Parallax High speed continuous servo (x2)


## 最適速度旋回アルゴリズム

### 2022 10/31
左右のモーター出力を，ロボットの速度 $v$ と，旋回角速度 $\omega$ を用いて制御するアルゴリズム(ovt.py).
カメラで捕らえた先行ロボットとの距離を $x$ ，進行方向との相対角度を $\theta$ としたとき，
下記の式で $v, \omega$ を更新します．

$$ \dot{v} =  a [ V(x)-v ]$$

$$ v' = v + \dot{v} dt $$

$$ \dot{\omega} =  a [\theta-\omega]$$

$$ \omega' = \omega + \dot{\omega} dt $$


ただし， $a$ は感応度， $V(x)$ が最適速度関数です．
$a$ を大きくするとロボットは敏感に変化に反応します．

最適速度関数 $V(x)$ は下記の式で求めます．

$$ V(x)=(1+\cos \theta)\alpha[\tanh(\beta(x-b)+c] $$

$\alpha$ は最高速度を決めるパラメータです． 
$b$ は安全距離です．

パラメータ $c$ は $0 \leq c \leq 1$ の範囲の値をとります． 
$c=1$と選ぶと，前進だけ行い，後退しないロボットになります．


左右のモーター出力 $L,R$ を

$$ L = v + d \omega $$

$$ R = v - d \omega $$

で求めます．

### 2021 5/26
 * omega = atanh(vx/vy)
 * vL= v + r * omega
 * vR= v - r * omega





 

