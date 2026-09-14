#!/usr/bin/env bash

# 进入项目根目录，保证 Python 可以正确导入 control 和 config 包。
cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1

# 优先使用现有飞控项目虚拟环境；其中已经安装 pyserial 和 pymavlink。
if [[ -f /home/orangepi/flydrive-env/bin/activate ]]; then
    # shellcheck disable=SC1091
    source /home/orangepi/flydrive-env/bin/activate
fi

# 使用 exec 让退出信号直接传递给 Python 控制程序。
exec python3 run_manual_control.py "$@"
