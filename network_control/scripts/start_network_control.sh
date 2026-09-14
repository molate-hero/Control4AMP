#!/usr/bin/env bash

# 进入网络控制模块根目录，保证 Python 可以找到 server.py 和资源目录。
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1

# 复用 Orange Pi 上已有的项目虚拟环境。
if [[ -f /home/orangepi/flydrive-env/bin/activate ]]; then
    # shellcheck disable=SC1091
    source /home/orangepi/flydrive-env/bin/activate
fi

# RealSense 的 numpy/pyrealsense2 安装在 Orange Pi 的用户 site-packages，
# 网络控制仍使用虚拟环境中的 Flask、pyserial 和 pymavlink；两部分需要同时可见。
export PYTHONPATH="/home/orangepi/.local/lib/python3.10/site-packages:${PYTHONPATH:-}"

# 默认仿真；需要真实硬件时显式添加 --hardware。
exec python3 run_network_control.py "$@"
