
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


## History

### 2022 10/31
左右のモーター出力の更新アルゴリズム(ovt.py).
カメラで捕らえた先行ロボットとの距離を $d$ ，進行方向との相対角度を $\theta$ としたとき，

$$ v' = v + a [ V(d)-v ]$$

$$ \omega' = \omega + a [\theta-\omega]$$

ただし， $V(d)$ が最適速度です．

$$ V(d)=(1+\cos \theta)\alpha[\tanh(\beta(d-b)+c]


### 2021 5/26
 * omega = atanh(vx/vy)
 * vL= v + r * omega
 * vR= v - r * omega





 

