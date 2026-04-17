#!/usr/bin/env bash
# Executa o SimBridge + ROS2Agent sourciando o workspace correto do ROS2.
# Uso: ./scripts/run_ros2.sh [--viewer] [--duration 60] [--speed 1.0]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# ── encontrar workspace ROS2 ──────────────────────────────────────────────────
HSL_SETUP="$HOME/Documents/hl_unification/hsl-player/install/setup.bash"
ROS_SETUP="/opt/ros/humble/setup.bash"

if [ -f "$HSL_SETUP" ]; then
    echo "· sourcing hsl-player workspace: $HSL_SETUP"
    # shellcheck disable=SC1090
    source "$HSL_SETUP"
elif [ -f "$ROS_SETUP" ]; then
    echo "· sourcing ROS2 Humble: $ROS_SETUP"
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
else
    echo "ERRO: ROS2 não encontrado."
    echo "      Instale o hsl-player ou ROS2 Humble e tente novamente."
    exit 1
fi

# ── garantir que o projeto está no PYTHONPATH ─────────────────────────────────
export PYTHONPATH="$PROJECT_DIR${PYTHONPATH:+:$PYTHONPATH}"

echo "· PYTHONPATH=$PYTHONPATH"
echo "· Iniciando sim-ros2…"
exec python "$SCRIPT_DIR/run_ros2.py" "$@"
