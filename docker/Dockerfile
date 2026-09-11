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
#  O contexto nomeado `hsl` usa robocin/hsl-player@main26 por padrão.
#  BuildKit clona o repositório; HSL_PLAYER_DIR permite um clone local.
#  O patch versionado de kick_intent publica a intenção do Kick normal.
#
#  Build:
#      docker compose -f docker/compose.yaml build
#
#  Estágios:
#      base   ROS2 Humble + usuário booster + perfil FastDDS
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
        build-essential cmake git curl wget unzip patch ca-certificates openssh-client \
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

# Qual commit do hsl-player foi copiado.  O compose preenche com
# HSL_COMMIT=$(git -C ~/Documents/hsl-player rev-parse --short HEAD).
ARG HSL_COMMIT=desconhecido
ENV HSL_COMMIT=${HSL_COMMIT}
LABEL org.opencontainers.image.revision=${HSL_COMMIT}

# ────────────────────────────────────────────────────────────── source ───────
# Espelha só o src/ da fonte main26 (GitHub ou clone local).  build/, install/ e log/ ficam na raiz do
# repo do hsl-player, então nem entram aqui.  Os estágios seguintes montam este
# como bind e copiam pacote a pacote — nada dele vira camada da imagem final.
FROM base AS source
COPY --from=hsl src /hsl-src

# ──────────────────────────────────────────────────────────────── msgs ───────
# Só as 4 definições de mensagem.  É o que o sim precisa, e a base do robô.
FROM base AS msgs

WORKDIR ${HSL_DIR}

# Os pacotes são localizados pelo NOME declarado no package.xml, não pelo
# caminho: o layout do hsl-player muda entre branches (competition-alan tem
# src/booster_ros2_interface e src/robocup_ros2_interface; sim_stable põe as
# quatro interfaces em src/interface/).  Nome de diretório também ≠ nome de
# pacote — src/booster_ros2_interface declara `booster_interface`.
#
# O bind mount evita que os 25 MB de src/vision/thirdparty virem camada.
RUN --mount=type=bind,from=source,source=/hsl-src,target=/hsl-src \
    set -eux; \
    mkdir -p src; \
    for pkg in booster_msgs booster_interface vision_interface game_controller_interface; do \
        f=$(grep -rl "<name>$pkg</name>" /hsl-src --include=package.xml | head -1); \
        [ -n "$f" ] || { echo "ERRO: pacote '$pkg' não encontrado no hsl-player" >&2; exit 1; }; \
        cp -r "$(dirname "$f")" "src/$pkg"; \
    done; \
    ls src

# Alguns branches do hsl-player (sim_stable, por ex.) não chamam
# find_package(rosidl_default_generators) nos pacotes de mensagem da Booster.
# Sem isso o rosidl_generate_interfaces gera só C/C++, e o sim morre com
# "ModuleNotFoundError: No module named 'booster_msgs'" — o brain é C++ e nunca
# sentiu falta, mas o hl_sim é Python.
#
# O patch é aditivo: liga o gerador Python e não toca em nenhuma definição .msg,
# então os tipos no fio continuam idênticos aos que o brain usa.
RUN set -eux; \
    for pkg in booster_msgs booster_interface vision_interface game_controller_interface; do \
        cml="src/$pkg/CMakeLists.txt"; px="src/$pkg/package.xml"; \
        if grep -q "rosidl_generate_interfaces" "$cml" \
           && ! grep -q "find_package(rosidl_default_generators" "$cml"; then \
            sed -i 's|find_package(ament_cmake REQUIRED)|find_package(ament_cmake REQUIRED)\nfind_package(rosidl_default_generators REQUIRED)|' "$cml"; \
            echo "· $pkg: bindings Python habilitadas"; \
        fi; \
        grep -q "rosidl_default_runtime" "$px" || \
            sed -i 's|</package>|  <exec_depend>rosidl_default_runtime</exec_depend>\n</package>|' "$px"; \
    done

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
        libeigen3-dev libopencv-dev libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Bug de empacotamento do ros-humble-behaviortree-cpp (4.9.1): o .deb instala a
# lib em lib/x86_64-linux-gnu/, mas o ament_cmake_export_libraries-extras.cmake
# dele só procura em <prefix>/lib.  Sem este symlink o find_package(behaviortree_cpp)
# do brain (CMakeLists.txt:14) falha com "exports the library 'behaviortree_cpp'
# which couldn't be found".
RUN set -eux; \
    for so in /opt/ros/humble/lib/*/libbehaviortree_cpp.so*; do \
        [ -e "$so" ] && ln -sf "$so" /opt/ros/humble/lib/; \
    done; \
    ls -l /opt/ros/humble/lib/libbehaviortree_cpp.so

# ── Rerun C++ SDK ────────────────────────────────────────────────────────────
# O brain faz find_package(rerun_sdk REQUIRED) (src/brain/CMakeLists.txt:16) e o
# SDK não está no apt.  O hsl-player não pina versão em lugar nenhum; 0.21.0 foi
# recuperada das imagens t1-demo e hl-unification, que compilaram este brain.
# Não suba sem testar: a partir da 0.22 o RecordingStream perdeu connect(), que
# brain_log.cpp:14 usa, e os archetypes mudaram (static_assert em AsComponents).
# Sem libarrow-dev: ele não existe no jammy (é do repositório da Apache Arrow), e
# o CMake do SDK do Rerun já baixa e compila o Arrow sozinho por padrão
# (RERUN_DOWNLOAD_AND_BUILD_ARROW=ON).  É lento, mas não depende de repo extra.
ARG RERUN_VERSION=0.21.0
RUN wget -q -O /tmp/rerun.zip \
        "https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip" \
    && unzip -q /tmp/rerun.zip -d /tmp/rerun \
    && cmake -S /tmp/rerun/rerun_cpp_sdk -B /tmp/rerun/build \
             -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/rerun/build --target install -j"$(nproc)" \
    && rm -rf /tmp/rerun /tmp/rerun.zip

# ── SDK da Booster Robotics ──────────────────────────────────────────────────
# brain.h:16 faz #include <booster/robot/b1/b1_api_const.hpp>.  Esse header não
# está no hsl-player nem em pacote apt nenhum: vem do SDK da Booster, que no robô
# real está instalado no sistema.  O README do upstream (BoosterRobotics/
# robocup_demo) só diz "install latest Booster Robotics SDK", sem versão.
#
# O brain usa só constantes do SDK e não linka a biblioteca (o CMakeLists dele
# não referencia nenhum lib da Booster), então bastam os headers.
ARG BOOSTER_SDK_REF=main
RUN git clone --depth 1 --filter=blob:none --sparse --progress \
        --branch "${BOOSTER_SDK_REF}" \
        https://github.com/BoosterRobotics/booster_robotics_sdk.git /tmp/bsdk \
 && cd /tmp/bsdk && git sparse-checkout set include \
 && cp -r /tmp/bsdk/include/booster /usr/local/include/ \
 && rm -rf /tmp/bsdk


WORKDIR ${HSL_DIR}

# Mesmo esquema do estágio msgs: localizados pelo nome do pacote.
# Do `vision` levamos só os YAML — o nó não sobe (precisa de PCL/ZED e disputaria
# /booster_vision/detection com o sim), mas brain/launch/launch.py:12 lê
# vision/share/vision/config/vision.yaml na hora do launch.
RUN --mount=type=bind,from=source,source=/hsl-src,target=/hsl-src \
    set -eux; \
    for pkg in brain game_controller; do \
        f=$(grep -rl "<name>$pkg</name>" /hsl-src --include=package.xml | head -1); \
        [ -n "$f" ] || { echo "ERRO: pacote '$pkg' não encontrado" >&2; exit 1; }; \
        cp -r "$(dirname "$f")" "src/$pkg"; \
    done; \
    vf=$(grep -rl "<name>vision</name>" /hsl-src --include=package.xml | head -1); \
    mkdir -p /tmp/vision-config; \
    cp "$(dirname "$vf")"/config/*.yaml /tmp/vision-config/; \
    ls src /tmp/vision-config


# ── SDK INTERNO da Booster (condicional) ─────────────────────────────────────
# Alguns branches do hsl-player incluem <booster_internal/robot/b1/...>, que não
# existe em repositório público nenhum — é o SDK que a Booster entrega aos times.
# Ele entrou junto com o "RL Kick" (ago/2025) e está em competition-alan; branches
# como sim_stable e main não usam.  Por isso a exigência é derivada do código
# copiado, não assumida.  Ver vendor/README.md.
#
# Duas fontes, nesta ordem:
#   vendor/booster_internal/       headers de verdade, copiados do robô (gitignored)
#   vendor/booster_internal.stub/  stub versionado, só api_ids conhecidos + tipos
# O stub existe para a imagem de SIMULAÇÃO compilar sem o SDK proprietário; ele
# tem quatro api_ids inventados, e por isso a imagem sai marcada.
COPY --chown=booster:booster vendor/ /tmp/vendor/
RUN set -eu; \
    if grep -rq "booster_internal" src/brain/ 2>/dev/null; then \
        if [ -d /tmp/vendor/booster_internal ]; then \
            cp -r /tmp/vendor/booster_internal /usr/local/include/; \
            echo "· este brain usa booster_internal — headers reais, de vendor/"; \
        elif [ -d /tmp/vendor/booster_internal.stub ]; then \
            cp -r /tmp/vendor/booster_internal.stub /usr/local/include/booster_internal; \
            touch /usr/local/include/booster_internal/.STUB; \
            echo "· ATENÇÃO: booster_internal veio do STUB (vendor/booster_internal.stub/)."; \
            echo "  Imagem válida para simulação; NÃO instale no robô real."; \
        else \
            echo "" >&2; \
            echo "ERRO: este branch do brain inclui <booster_internal/...>, mas" >&2; \
            echo "      nem vendor/booster_internal/ nem vendor/booster_internal.stub/ existem." >&2; \
            echo "" >&2; \
            echo "  Três saídas:" >&2; \
            echo "  1) copiar os headers do robô real:" >&2; \
            echo "       scp -r booster@<ip>:/usr/local/include/booster_internal vendor/" >&2; \
            echo "  2) restaurar o stub versionado (git checkout vendor/booster_internal.stub)" >&2; \
            echo "  3) buildar um branch que não precisa dele:" >&2; \
            echo "       git -C ~/Documents/hsl-player worktree add ../hsl-player-sim sim_stable" >&2; \
            echo "       HSL_PLAYER_DIR=../../hsl-player-sim docker compose ... build" >&2; \
            echo "" >&2; \
            echo "  Detalhes em vendor/README.md" >&2; \
            exit 1; \
        fi; \
    else \
        echo "· este brain não usa booster_internal — nada a instalar"; \
    fi; \
    rm -rf /tmp/vendor

# O Kick normal só manda velocidade. Publicar sua intenção permite que o
# consumidor de locomoção execute o chute sem inferi-lo de uma aproximação.
# Patch estrito: alterações incompatíveis no upstream fazem o build falhar.
COPY docker/patches/main26-kick-intent.patch /tmp/main26-kick-intent.patch
RUN patch --batch --forward --fuzz=0 -p1 < /tmp/main26-kick-intent.patch

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
