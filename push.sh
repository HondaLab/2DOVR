#!/usr/bin/sh
# pushを自動化するスクリプト
file=2dovr.py
comment="tof値をudpで受け取る"
git add $file
sleep 2
git commit -m "$comment"
sleep 2
git push
