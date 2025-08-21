#!/bin/bash

DEV1=/dev/v4l/by-id/usb-SHENZHEN_AONI_ELECTRONIC_CO.__LTD_Full_HD_webcam_AN202011030002-video-index0
DEV2=/dev/v4l/by-id/usb-icSpring_WebCamera_20240603201703-video-index0
DEV3=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0

ffmpeg -f v4l2 -input_format mjpeg -framerate 10 -video_size 1280x720 -i "$DEV1" -f mpjpeg -q:v 5 -listen 1 http://0.0.0.0:8081/cam.mjpg
ffmpeg -f v4l2 -input_format mjpeg -framerate 10 -video_size 1280x720 -i "$DEV2" -f mpjpeg -q:v 5 -listen 1 http://0.0.0.0:8082/cam.mjpg
ffmpeg -f v4l2 -input_format mjpeg -framerate 10 -video_size 1280x720 -i "$DEV3" -f mpjpeg -q:v 5 -listen 1 http://0.0.0.0:8083/cam.mjpg