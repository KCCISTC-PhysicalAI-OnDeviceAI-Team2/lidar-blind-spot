#!/usr/bin/env bash
# 평가 · 샘플 rosbag 녹화: 정해진 토픽 세 개를 ~/bags/<이름>에 녹화한다.
# 사용법: record_bag.sh <이름> [초, 기본 13]
#   예: record_bag.sh 20261027_sample 12
# 13초: 앞부분 약 2.5초는 카메라 구독 전이라 평가에서 빼도 10초 이상 남게 한다.
# 센서는 먼저 켜 둔다: ros2 launch blindspot_bringup sensors.launch.py
set -eu

if [ $# -lt 1 ]; then
  echo "사용법: $0 <이름> [초]"
  exit 1
fi

name="$1"
seconds="${2:-13}"
out="$HOME/bags/$name"
topics=(/scan /camera/image_raw/compressed /tf_static)

if [ -e "$out" ]; then
  echo "이미 있음: $out"
  exit 1
fi
mkdir -p "$HOME/bags"

echo "녹화 ${seconds}초 -> $out"
echo "토픽: ${topics[*]}"
# 시간이 되면 Ctrl+C와 같은 신호(INT)로 멈춰 rosbag을 정상 종료한다.
timeout --foreground -s INT "$seconds" ros2 bag record -o "$out" "${topics[@]}" || true

ros2 bag info "$out"
