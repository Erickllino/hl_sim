#!/usr/bin/env bash
# Executa o simulador sourciando o workspace ROS2 correto.
#
# Este script é para rodar o sim direto no host (com ROS2 instalado).  O caminho
# padrão do projeto é o Docker — ver docker/compose.yaml.
#
# Uso: ./tools/run_ros2.sh [--brains 1,3] [--viewer] [--duration 60]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# ── workspace ROS2 ────────────────────────────────────────────────────────────
HSL_SETUP="${HSL_PLAYER_DIR:-$HOME/Workspace/hsl-player}/install/setup.bash"
ROS_SETUP="/opt/ros/humble/setup.bash"

if [ -f "$HSL_SETUP" ]; then
    echo "· sourcing workspace do hsl-player: $HSL_SETUP"
    # shellcheck disable=SC1090
    source "$HSL_SETUP"
elif [ -f "$ROS_SETUP" ]; then
    echo "· sourcing ROS2 Humble: $ROS_SETUP"
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
else
    echo "ERRO: ROS2 não encontrado."
    echo "      Builde o hsl-player, instale o ROS2 Humble, ou use o Docker."
    exit 1
fi

# ── ambiente do hl_sim ────────────────────────────────────────────────────────
# src-layout: o que entra no PYTHONPATH é src/, não a raiz do repo.
export PYTHONPATH="$PROJECT_DIR/src${PYTHONPATH:+:$PYTHONPATH}"
export HL_SIM_ROOT="$PROJECT_DIR"

if [ -f "$PROJECT_DIR/.venv/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$PROJECT_DIR/.venv/bin/activate"
fi

echo "· HL_SIM_ROOT=$HL_SIM_ROOT"
exec python -m hl_sim.cli.run_ros2 "$@"
