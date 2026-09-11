#!/usr/bin/env bash
# Container do simulador: ROS2 + as 4 interfaces do hsl-player + venv do hl_sim.
set -eo pipefail

# Sem `set -u` aqui: os setup.bash do ROS2 leem AMENT_TRACE_SETUP_FILES sem
# definir, e com -u o entrypoint morre na primeira linha, antes de qualquer log.
set +u
source /opt/ros/humble/setup.bash
source "${HSL_DIR}/install/setup.bash"
set -u

# O venv tem --system-site-packages, então rclpy continua visível depois disto.
if [ -f /opt/venv/bin/activate ]; then
    source /opt/venv/bin/activate
fi

# src-layout: o que entra no PYTHONPATH é /workspace/src, não /workspace — assim
# config/, docker/ e tools/ não viram nomes de topo importáveis.
export PYTHONPATH="/workspace/src${PYTHONPATH:+:$PYTHONPATH}"
export HL_SIM_ROOT=/workspace

if [ -n "${DISPLAY:-}" ]; then
    echo "· viewer: DISPLAY=${DISPLAY}  MUJOCO_GL=${MUJOCO_GL:-glfw}"
else
    echo "· sem DISPLAY — rode com --viewer só se tiver feito 'xhost +local:docker'"
fi

exec "$@"
