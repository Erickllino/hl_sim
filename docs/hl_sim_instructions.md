# Briefing: Novo Repo `hl_sim`

## O que é

Repositório independente que cria um ambiente de simulação MuJoCo para robôs T1
jogarem futebol usando a stack **exata** do `hsl_player` — sem modificar nada no
brain, apenas substituindo o hardware por simulação.

---

## Contexto: Repos existentes

| Repo                            | Papel                                                                       |
| ------------------------------- | --------------------------------------------------------------------------- |
| `hl_unification/booster_deploy` | Deploy no robô real. No futuro: entrada de modelos RL na lógica estratégica |
| `hl_unification/hsl_player`     | Brain + behavior tree + visão. Roda no robô e no sim sem modificações       |
| **`hl_sim`** (novo)             | Simulação MuJoCo. Imita a interface do hardware para o hsl_player           |

---

## Ideia Central

```
[hsl_player brain]  ←→  ROS2  ←→  [sim_bridge.py]  ←→  MuJoCo
       ↑                                  ↑
  sem modificação                 novo repo (aqui)
```

O `hsl_player` não sabe que está falando com um simulador.
O sim publica os mesmos tópicos ROS2 que o hardware real publicaria.

---

## Por que repo separado (não dentro do booster_deploy)

- `booster_deploy` tem dependências do SDK do Booster Robotics (hardware-only)
- O sim vai crescer com assets MuJoCo, cenas XML, scripts de treino RL
- Pode rodar em qualquer máquina sem instalar drivers do robô
- Separação limpa: sim = environment de treino/teste; deploy = produção no hardware

---

## Arquitetura

```
t1_soccer_sim/
├── scenes/
│   ├── soccer_field.xml          # campo RoboCup HL (9m × 6m) — JÁ EXISTE (trazer do hl_unification)
│   └── soccer_scene.xml          # campo + bola + N robôs T1
├── bridge/
│   ├── sim_bridge.py             # MuJoCo ↔ ROS2 (instância por robô)
│   └── locomotion_runner.py      # roda t1_walk.pt internamente no sim
├── mocks/
│   ├── vision_mock.py            # ground truth MuJoCo como booster_vision
│   └── game_controller_mock.py   # estados do jogo: INITIAL→READY→SET→PLAYING
├── launch/
│   ├── single_robot.launch.py    # 1 robô para desenvolvimento
│   └── soccer_game.launch.py     # N robôs, jogo completo
├── models/
│   └── t1_walk.pt                # cópia da locomotion policy (de booster_deploy)
├── assets/
│   └── T1_23dof.xml              # modelo MuJoCo do T1 (de booster_assets)
├── pyproject.toml
└── README.md
```

---

## Tópicos ROS2 que o sim publica (imitando o hardware)

| Tópico                      | Fonte                            | Tipo                               |
| --------------------------- | -------------------------------- | ---------------------------------- |
| `/low_state`                | `mj_data.qpos/qvel`              | `booster_msgs/LowState`            |
| `/odometer_state`           | `mj_data.qpos[0:3]` + quaternion | `booster_msgs/OdometerState`       |
| `/booster_vision/detection` | ground truth do MuJoCo           | `vision_msgs/Detection`            |
| `/booster_vision/ball`      | `mj_data.body("ball").xpos`      | `vision_msgs/BallDetection`        |
| `/robocup/game_controller`  | state machine interna            | `booster_msgs/GameControllerState` |

## Tópicos ROS2 que o sim assina (comandos do hsl_player)

| Tópico                                      | Ação no MuJoCo                              |
| ------------------------------------------- | ------------------------------------------- |
| `LocoApiTopicReq` (api_id=2001 kMove)       | atualiza cmd_vel da locomotion policy       |
| `LocoApiTopicReq` (api_id=2004 kRotateHead) | move joints da cabeça                       |
| `LocoApiTopicReq` (api_id=2024 kShoot)      | aplica força na bola ou reproduz trajetória |

---

## Fases de Implementação

```
Fase 1 — Cena MuJoCo
  - soccer_scene.xml: campo + bola + 1 robô T1
  - testar com mujoco viewer

Fase 2 — Bridge (robô único)
  - sim_bridge.py: loop MuJoCo + publicação de /low_state + /odometer_state
  - locomotion_runner.py: roda t1_walk.pt, aplica torques

Fase 3 — Vision Mock
  - vision_mock.py: calcula o que o robô "enxerga" via geometria do MuJoCo
  - publica /booster_vision/detection e /booster_vision/ball

Fase 4 — Game Controller Mock
  - game_controller_mock.py: INITIAL → READY → SET → PLAYING → [gol → reset]
  - detecta gol por ball.xpos[0] cruzando linha do gol

Fase 5 — Chute
  - Fase 5a (proto): força direta na bola via xfrc_applied
  - Fase 5b (cinemática): trajetória t1_kick.npz reproduzida frame a frame

Fase 6 — Multi-robô
  - N instâncias de sim_bridge com --robot-id N
  - Cada brain em namespace /robot_N/

Fase 7 — Launch unificado
  - ros2 launch t1_soccer_sim soccer_game.launch.py num_robots:=4
```

---

## Dependências do novo repo

Sem pacotes novos. Tudo já usado no projeto:

- `mujoco` — física
- `rclpy` + `booster_msgs` — ROS2
- `torch` — locomotion policy
- `numpy` — geometria e transformações

**Dependência externa (só para rodar o brain):**
- `hsl_player` instalado/buildado em ROS2 workspace separado (não é dependência de código, só de runtime via ROS2)

---

## Relação com RL (futuro)

```
Treino RL
  └── usa t1_soccer_sim como gym environment (gym wrapper sobre o sim)
  └── modelo treinado → exportado para hsl_player ou booster_deploy
  └── validado no t1_soccer_sim antes de ir pro hardware
```

O sim vira o **ambiente de treino e validação**.
O `booster_deploy` recebe os modelos prontos para rodar no hardware real.

---

## Assets a copiar do hl_unification

- `booster_assets/robots/T1/T1_23dof.xml` → `t1_soccer_sim/assets/T1_23dof.xml`
- `booster_deploy/tasks/locomotion/models/policy.pt` → `t1_soccer_sim/models/t1_walk.pt`
- XML do campo (trazer para `t1_soccer_sim/scenes/soccer_field.xml`)

---

## Primeiro passo ao abrir o repo novo

1. Criar `soccer_scene.xml` incluindo `soccer_field.xml` + `T1_23dof.xml` + bola
2. Testar cena: `python -m mujoco.viewer --mjcf=scenes/soccer_scene.xml`
3. Criar `sim_bridge.py` com loop básico publicando `/low_state`
4. Confirmar que `hsl_player` recebe os tópicos e não crasha
