# syntax=docker/dockerfile:1.7
# ==============================================================================
#  hl_sim — duas imagens, um Dockerfile
#
#     hl-robot : ROS2 Humble + hsl-player (brain + game_controller).  6 containers.
#     hl-sim   : ROS2 Humble + as 4 interfaces + MuJoCo + hl_sim.     1 container.
#
#  Princípio: o container do robô NÃO sabe que existe um simulador.  Ele não tem
#  mujoco, não tem hl_sim, não tem a cena.  Se soubesse, a fidelidade que é o
#  objetivo do projeto estaria perdida.  O caminho do workspace é o mesmo do robô
#  real — /home/booster/Workspace/hsl-player — para os logs e paths baterem.
#
#  Build (BuildKit obrigatório; o clone usa a sua chave SSH, que nunca entra numa
#  camada da imagem):
#      docker compose -f docker/compose.yaml build
#  ou, à mão:
#      DOCKER_BUILDKIT=1 docker build --ssh default \
#          -f docker/Dockerfile --target robot -t hl-robot:humble .
#
#  Estágios:
#      base   ROS2 Humble + usuário booster + perfil FastDDS
#      fetch  clone do hsl-player, sem os artefatos de build versionados
#      msgs   colcon build só das 4 interfaces        (compartilhado)
#        ├─ sim    + uv/MuJoCo + hl_sim
#        └─ robot  + rerun/behaviortree/OpenCV + colcon build brain e game_controller
# ==============================================================================

# ──────────────────────────────────────────────────────────────── base ───────
FROM ros:humble-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_DISTRO=humble

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl wget unzip ca-certificates openssh-client \
        python3-pip python3-venv python3-colcon-common-extensions \
        ros-humble-rmw-fastrtps-cpp \
        iproute2 iputils-ping net-tools tcpdump \
    && rm -rf /var/lib/apt/lists/*

# Mesmo usuário e caminho do robô real: /home/booster/Workspace/hsl-player
# (confirmado nos CMakeCache.txt versionados do hsl-player).
# UID/GID batem com o do host para os bind mounts não virarem root-owned.
ARG UID=1000
ARG GID=1000
RUN set -eux; \
    if getent passwd "${UID}" >/dev/null; then \
        existing="$(getent passwd "${UID}" | cut -d: -f1)"; \
        usermod -l booster -d /home/booster -m "${existing}"; \
        groupmod -n booster "$(id -gn booster)" || true; \
    else \
        groupadd -g "${GID}" booster || true; \
        useradd -m -u "${UID}" -g "${GID}" -s /bin/bash booster; \
    fi; \
    mkdir -p /home/booster/Workspace /etc/hl; \
    chown -R booster:booster /home/booster /etc/hl

# O configs/fastdds.xml do hsl-player tem interfaceWhiteList com 127.0.0.1 e os
# IPs internos do robô real (192.168.10.101/102) — dentro do container isso
# bloqueia a bridge.  Este perfil é o equivalente para o container.
COPY docker/fastdds-container.xml /etc/hl/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/hl/fastdds.xml \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    PYTHONUNBUFFERED=1

ENV HSL_DIR=/home/booster/Workspace/hsl-player

# ─────────────────────────────────────────────────────────────── fetch ───────
FROM base AS fetch

ARG HSL_REPO=git@github.com:BoosterRobotics/robocup_demo.git
ARG HSL_REF=sandbox/support_2026_game_controller

RUN mkdir -p -m 0700 /root/.ssh && ssh-keyscan github.com >> /root/.ssh/known_hosts

# HSL_REF entra como ARG para invalidar o cache quando o brain mudar de commit.
RUN --mount=type=ssh \
    git clone --depth 1 --branch "${HSL_REF}" "${HSL_REPO}" /src/hsl-player

# O hsl-player versiona build/ install/ log/ — 4876 dos 5294 arquivos rastreados,
# compilados no Jetson (arm64).  Deixar isso no workspace faz o colcon achar
# caches do CMake com compilador e caminhos absolutos errados.
RUN rm -rf /src/hsl-player/build /src/hsl-player/install /src/hsl-player/log \
           /src/hsl-player/.git

# ──────────────────────────────────────────────────────────────── msgs ───────
# Só as 4 definições de mensagem.  É o que o sim precisa, e a base do robô.
FROM base AS msgs

WORKDIR ${HSL_DIR}

COPY --from=fetch --chown=booster:booster /src/hsl-player/src/booster_msgs           src/booster_msgs
COPY --from=fetch --chown=booster:booster /src/hsl-player/src/booster_ros2_interface src/booster_ros2_interface
COPY --from=fetch --chown=booster:booster /src/hsl-player/src/robocup_ros2_interface src/robocup_ros2_interface

# Nome do diretório ≠ nome do pacote: src/booster_ros2_interface → booster_interface.
RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install --parallel-workers "$(nproc)" \
        --packages-select booster_msgs booster_interface \
                          vision_interface game_controller_interface \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && chown -R booster:booster ${HSL_DIR}

# ─────────────────────────────────────────────────────────────── robot ───────
FROM msgs AS robot

# Dependências que só o brain usa.  Nada disto entra na imagem do sim.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-behaviortree-cpp \
        ros-humble-backward-ros \
        ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
        libeigen3-dev libopencv-dev libyaml-cpp-dev libarrow-dev \
    && rm -rf /var/lib/apt/lists/*

# ── Rerun C++ SDK ────────────────────────────────────────────────────────────
# O brain faz find_package(rerun_sdk REQUIRED) (src/brain/CMakeLists.txt:16) e o
# SDK não está no apt.  O hsl-player não pina versão em lugar nenhum, então este
# é o candidato nº 1 a quebrar o build: se o compilador reclamar da API do Rerun,
# mude RERUN_VERSION (docker/compose.yaml → build.args).
ARG RERUN_VERSION=0.36.3
RUN wget -q -O /tmp/rerun.zip \
        "https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip" \
    && unzip -q /tmp/rerun.zip -d /tmp/rerun \
    && cmake -S /tmp/rerun/rerun_cpp_sdk -B /tmp/rerun/build \
             -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/rerun/build --target install -j"$(nproc)" \
    && rm -rf /tmp/rerun /tmp/rerun.zip

WORKDIR ${HSL_DIR}

COPY --from=fetch --chown=booster:booster /src/hsl-player/src/game_controller src/game_controller
COPY --from=fetch --chown=booster:booster /src/hsl-player/src/brain           src/brain
# Do pacote vision levamos só os YAML: o nó não sobe (precisa de PCL/ZED e
# disputaria /booster_vision/detection com o sim), mas brain/launch/launch.py:12
# lê vision/share/vision/config/vision.yaml na hora do launch.
COPY --from=fetch --chown=booster:booster /src/hsl-player/src/vision/config   /tmp/vision-config

RUN . /opt/ros/humble/setup.sh \
    && . ${HSL_DIR}/install/setup.sh \
    && colcon build --symlink-install --parallel-workers "$(nproc)" \
        --packages-select game_controller brain \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && mkdir -p ${HSL_DIR}/install/vision/share/vision/config \
    && cp /tmp/vision-config/*.yaml ${HSL_DIR}/install/vision/share/vision/config/ \
    && rm -rf /tmp/vision-config \
    && chown -R booster:booster ${HSL_DIR}

# Launch que sobe game_controller_node + brain_node no mesmo container — é o
# par que roda junto no robô real, e os dois precisam do mesmo ROS_DOMAIN_ID.
COPY --chown=booster:booster docker/launch/robot.launch.py /etc/hl/robot.launch.py
COPY docker/entrypoint-robot.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER booster
WORKDIR ${HSL_DIR}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["ros2", "launch", "/etc/hl/robot.launch.py"]

# ───────────────────────────────────────────────────────────────── sim ───────
FROM msgs AS sim

# Libs de GL para o viewer do MuJoCo (glfw via X11/XWayland).
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglew2.2 libegl1 libglfw3 libosmesa6 \
        libx11-6 libxext6 libxrender1 libxrandr2 libxinerama1 libxcursor1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:latest /uv /usr/local/bin/uv

# --system-site-packages é obrigatório: rclpy vem do apt e não é pip-instalável.
# Também é por isso que o pyproject deixa numpy permissivo — rclpy e cv_bridge
# do Humble foram compilados contra a ABI do numpy 1.x.
ENV UV_PROJECT_ENVIRONMENT=/opt/venv \
    UV_LINK_MODE=copy \
    PATH=/opt/venv/bin:$PATH
RUN uv venv --system-site-packages --python python3.10 /opt/venv \
 && chown -R booster:booster /opt/venv

WORKDIR /workspace

# Camada de dependências separada do código: só reinstala quando o lock muda.
COPY --chown=booster:booster pyproject.toml uv.lock ./
RUN uv sync --inexact --no-install-project && chown -R booster:booster /opt/venv

# O código real vem por bind mount no compose; este COPY é para a imagem rodar
# sozinha (CI, outra máquina).
COPY --chown=booster:booster . /workspace

ENV HL_SIM_ROOT=/workspace \
    PYTHONPATH=/workspace/src \
    MUJOCO_GL=glfw

COPY docker/entrypoint-sim.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER booster
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["python", "-m", "hl_sim.cli.run_ros2", "--brains", "all"]
