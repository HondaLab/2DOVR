#!/usr/bin/sh
# pushを自動化するスクリプト
file=tof.py
comment="tof距離センサー用"
git add $file
sleep 2
git commit -m "$comment"
sleep 2
git push
