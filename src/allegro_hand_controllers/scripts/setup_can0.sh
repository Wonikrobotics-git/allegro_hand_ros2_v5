#!/bin/bash

# 다운시키기
sudo ip link set can0 down

# 비트레이트 설정
sudo ip link set can0 type can bitrate 1000000

# 인터페이스 업
sudo ip link set can0 up

# 상태 확인
#ip -details -statistics link show can0
