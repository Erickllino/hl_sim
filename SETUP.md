# hl_sim — Setup & Operação

## Estrutura de repos

| Repo | Papel |
|------|-------|
| `~/hl_sim` | Simulação MuJoCo: cena XML, bridge, agentes |
| `~/hl_unification/hsl-player` | Brain C++ (nó ROS2) + behavior trees + visão |
| `~/hl_unification/booster_robotics_sdk` | SDK público do Booster Robotics (DDS/RPC) |

---

## Pré-requisitos (uma única vez)

### 1. Symlink `/workspace`

O workspace do brain foi buildado dentro de um container Docker onde o repo estava montado em `/workspace`. Todos os symlinks `--symlink-install` do colcon apontam para esse caminho. Crie o symlink no host para que eles resolvam:

```bash
sudo ln -s /home/jefferson/hl_unification /workspace
```

### 2. Instalar dependências Python do sim

```bash
cd ~/hl_sim
uv sync
```

### 3. Buildar o workspace ROS2 (excluindo `vision` que precisa de CUDA)

```bash
source /opt/ros/humble/setup.bash
cd ~/hl_unification/hsl-player
colcon build --symlink-install --packages-skip vision
```

Pacotes que constroem com sucesso: `booster_msgs`, `booster_interface`, `vision_interface`, `game_controller_interface`, `game_controller`, `sound_play`.

O `brain` falha ao buildar nativamente porque depende de `booster_internal/robot/b1/b1_loco_internal_api.hpp` — um header privado do Booster que só existe no container Docker. O binário pré-compilado em `build/brain/brain_node` (gerado pelo Docker) é usado diretamente.

---

## Modo 1 — Simulação standalone (sem ROS2)

6 agentes com script autônomo (SEEK → ALIGN → APPROACH → KICK). Não precisa de ROS2 nem do brain.

```bash
cd ~/hl_sim
uv run python -m bridge.sim_bridge --viewer --duration 60
```

| Flag | Descrição |
|------|-----------|
| `--viewer` | Abre a janela gráfica MuJoCo |
| `--duration N` | Segundos de simulação (0 = infinito) |
| `--speed N` | Fator de velocidade real (0 = máximo, 1 = tempo real) |

---

## Modo 2 — Simulação com o brain (ROS2)

Abre dois terminais. Em ambos, source o ROS2 primeiro:

```bash
source /opt/ros/humble/setup.bash
source ~/hl_unification/hsl-player/install/setup.bash
```

**Terminal 1 — Brain:**

```bash
ros2 launch brain launch.py sim:=true deploy:=false
```

| Argumento | Valores | Descrição |
|-----------|---------|-----------|
| `sim:=true` | `true`/`false` | Ativa `use_sim_time` |
| `deploy:=false` | `true`/`false` | Não inicia o `booster_deploy` (walk controller de hardware) |
| `role:=striker` | `striker`/`goal_keeper` | Sobrescreve o role do `config.yaml` |
| `player_id:=1` | 1–5 | Sobrescreve o player ID |

**Terminal 2 — Sim:**

```bash
/home/jefferson/hl_sim/.venv/bin/python /home/jefferson/hl_sim/scripts/run_ros2.py --viewer
```

> Usar o Python do venv diretamente (não `uv run`) para que ele enxergue tanto os pacotes do venv (mujoco, numpy) quanto os pacotes ROS2 do ambiente sourced.

---

## Verificar comunicação

Com brain e sim rodando:

```bash
# Brain está no ar?
ros2 node list
# Deve mostrar: /brain_node

# Sim publica sensores?
ros2 topic hz /low_state
# Deve mostrar ~50 Hz

# Brain manda comandos de movimento?
ros2 topic echo LocoApiTopicReq
# Deve aparecer mensagens com api_id e vx/vy/vyaw quando o jogo está PLAYING
```

---

## Arquitetura do bridge (ROS2 mode)

```
[brain_node]  ←──────────────────────────────→  [run_ros2.py / ROS2Agent]  ←→  MuJoCo
     │                                                      │
     │  Assina:                                   Publica:
     │    LocoApiTopicReq  ←── vx/vy/vyaw ───────────────────
     │    /joint_ctrl      ←── joint targets ────────────────
     │                                            Assina:
     │  Publica:                                    /low_state (IMU + joints)
     │    LocoApiTopicReq ──────────────────────→   /odometer_state (x, y, theta)
     │                                              /booster_vision/detection (bola)
     │                                              /booster_vision/line_segments
     │                                              /robocup/game_controller
     │                                              /head_pose
```

---

## Arquitetura interna do sim

### Agentes

| Classe | Arquivo | Descrição |
|--------|---------|-----------|
| `DefaultAgent` | `bridge/default_agent.py` | Agente scripted: SEEK→ALIGN→APPROACH→KICK |
| `ROS2Agent` | `bridge/ros2_agent.py` | Ponte para o brain via ROS2 |

### Locomoção (kinematic)

O sim não roda uma policy de locomotion. Em vez disso:
- O tronco é teleportado integrando `vx, vy, vyaw` a cada tick.
- Os joints das pernas são animados por um **CPG sinusoidal** (`_gait_ctrl` em `sim_bridge.py`) baseado na velocidade comandada.

Para usar a policy RL real (`t1_walk.pt`), ver `ROADMAP_ROS2.md` Step 7.

### Constantes importantes (`sim_bridge.py`)

```python
CONTROL_HZ     = 50      # frequência dos agentes
SIM_HZ         = 200     # frequência do MuJoCo (4 passos por tick de controle)
TRUNK_HEIGHT   = 0.679   # altura do tronco na pose home
NUM_ROBOTS     = 6       # 3v3
```

---

## Problemas conhecidos

### Robot não se move no viewer (ROS2 mode)

**Sintoma:** brain publica `kMove` mas o robô fica parado.  
**Diagnóstico:**
```bash
ros2 topic echo LocoApiTopicReq   # confirmar que callbacks chegam
```
Adicionar log em `_on_loco` no `ros2_agent.py` para confirmar que o callback dispara.

### `vision_config_path not exists` ao rodar `ros2 run brain brain_node`

**Causa:** `ros2 run` não passa os parâmetros. Usar sempre `ros2 launch`.  
**Fix:** `ros2 launch brain launch.py sim:=true deploy:=false`

### `not found: .../local_setup.bash` ao fazer source

**Causa:** Symlink `/workspace` ausente.  
**Fix:** `sudo ln -s /home/jefferson/hl_unification /workspace`

### `ModuleNotFoundError: No module named 'booster_msgs'`

**Causa:** ROS2 não está sourced no terminal atual.  
**Fix:**
```bash
source /opt/ros/humble/setup.bash
source ~/hl_unification/hsl-player/install/setup.bash
```

### `vision` falha ao buildar (`Failed to find nvcc`)

**Causa:** `vision` usa TensorRT/CUDA para inferência no robô real. Não é necessário para simulação.  
**Fix:** `colcon build --packages-skip vision`
