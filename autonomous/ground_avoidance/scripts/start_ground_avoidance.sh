#!/usr/bin/env bash

# 自动避障模块默认只计算不发车；真实输出必须显式传入 --hardware。
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
# 复用 CONTROL 的串口依赖，并补充系统用户 site-packages 中的 RealSense SDK。
if [[ -f /home/orangepi/flydrive-env/bin/activate ]]; then
  # shellcheck disable=SC1091
  source /home/orangepi/flydrive-env/bin/activate
fi
export PYTHONPATH="/home/orangepi/.local/lib/python3.10/site-packages:${PYTHONPATH:-}"
exec python3 run_ground_avoidance.py "$@"
