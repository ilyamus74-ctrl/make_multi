#!/bin/bash

#set -e

#BRANCH="main"

#git fetch origin "$BRANCH"

#git add -A
#git commit -m "sync local -> remote" || true

#git push -u origin HEAD:"$BRANCH" --force-with-lease


cd ~/rknpu2/examples/rknn_yolov8_demo_opencv_2_work_2/new_yolo8
git init
git add -A
git commit -m "Initial commit from local"
git branch -M main
git remote add origin git@github.com:ilyamus74-ctrl/make_multi.git
git push -u origin main