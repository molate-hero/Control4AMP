#!/usr/bin/env bash

# VLM 视觉导航模块默认 dry-run；真实驱动小车必须显式传入 --hardware。
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

# 复用 CONTROL 的串口依赖（pyserial 在 flydrive-env 里），并补充系统用户
# site-packages 中的 RealSense SDK 与 numpy。
if [[ -f /home/orangepi/flydrive-env/bin/activate ]]; then
  # shellcheck disable=SC1091
  source /home/orangepi/flydrive-env/bin/activate
fi
export PYTHONPATH="/home/orangepi/.local/lib/python3.10/site-packages:${PYTHONPATH:-}"

exec python3 run_vlm_navigation.py "$@"
