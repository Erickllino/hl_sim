#!/usr/bin/env bash
# Container de um robô: sourceia ROS2 + workspace do hsl-player e instala o
# config_local.yaml deste robô antes de subir brain + game_controller.
set -eo pipefail

# Sem `set -u` aqui: os setup.bash do ROS2 leem AMENT_TRACE_SETUP_FILES sem
# definir, e com -u o entrypoint morre na primeira linha, antes de qualquer log.
set +u
source /opt/ros/humble/setup.bash
source "${HSL_DIR}/install/setup.bash"
set -u

# ── config_local.yaml ─────────────────────────────────────────────────────────
# brain/launch/launch.py:17 carrega config_local.yaml DEPOIS de config.yaml — é o
# gancho de override por máquina, e é onde entram team_id/player_id deste robô.
# O arquivo não existe no repo do hsl-player: é gerado no host por
# tools/gen_robot_configs.py a partir de config/match.yaml e montado em /etc/hl.
#
# Copiamos em vez de montar direto no share porque `colcon build --symlink-install`
# cria symlink por ARQUIVO, não por diretório: um arquivo novo criado depois do
# build não apareceria em share/brain/config.
BRAIN_CFG="${HSL_DIR}/install/brain/share/brain/config"
if [ -f /etc/hl/config_local.yaml ]; then
    cp /etc/hl/config_local.yaml "${BRAIN_CFG}/config_local.yaml"
    echo "· config_local.yaml instalado em ${BRAIN_CFG}"
    grep -E 'team_id|player_id|player_role' "${BRAIN_CFG}/config_local.yaml" | sed 's/^/    /'
else
    # O launch passa esse caminho como params-file; sem o arquivo o brain_node
    # morre com um erro que não diz o que faltou.
    echo "ERRO: /etc/hl/config_local.yaml não montado." >&2
    echo "      Rode:  python tools/gen_robot_configs.py" >&2
    exit 1
fi

echo "· ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION}"
exec "$@"
