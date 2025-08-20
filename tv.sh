#!/bin/bash

for n in /dev/v4l/by-id/usb-SHENZHEN_AONI_ELECTRONIC_CO.__LTD_Full_HD_webcam_AN202011030002-video-index*; do
  caps=$(udevadm info -q property -n "$n" | sed -n 's/^ID_V4L_CAPABILITIES=//p')
  echo "$n -> $caps"
done