#!/usr/bin/env bash
# 외부 USB 웹캠 기반 AprilTag 검출 PC의 의존성 설치.
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  ros-humble-apriltag-ros \
  ros-humble-camera-calibration \
  ros-humble-image-proc \
  ros-humble-v4l2-camera
