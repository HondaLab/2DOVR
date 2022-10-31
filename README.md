
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
カメラで捕らえた先行ロボットとの距離を $x$ ，進行方向との相対角度を $\theta$ としたとき，

$$ v' = v + a [ V(x)-v ]$$

$$ \omega' = \omega + a [\theta-\omega]$$

ただし， $a$ が感応度， $V(x)$ が最適速度です．
$v$ がロボットの速さ， $\omega$ がロボットの旋回角速度を表します．

最適速度 $V(x)$ は下記の式で求めます．

$$ V(x)=(1+\cos \theta)\alpha[\tanh(\beta(x-b)+c] $$

$\alpha$ は最高速度を決めるパラメータ， $b$ が安全距離です．

$-1 \leq c \leq 1$ のパラメータです．

左右のモーター出力 $L,R$ を

$$ L = v + l \omega $$

$$ R = v - l \omega $$

で求めます．

### 2021 5/26
 * omega = atanh(vx/vy)
 * vL= v + r * omega
 * vR= v - r * omega





 

