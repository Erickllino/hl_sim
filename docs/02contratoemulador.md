# 02 — Contrato do emulador

> O que o `hl_sim.emulator` precisa publicar e consumir, dentro do container
> do robô, para o `brain_node` do `robocup_demo` (sandbox
> `support_2026_game_controller`) rodar **sem saber que não está no robô**.
> Tudo aqui foi extraído do código do brain, dos pacotes de interface e do
> `booster_robotics_sdk`; onde há dúvida, o arquivo de origem está indicado.

Convenções: nomes de tópico são exatamente como aparecem em
`src/brain/src/brain.cpp:236-267` e `main.cpp:42-44`. Quando `robot.robot_name`
está vazio (caso do robô real e o nosso), o sufixo `/robotN` **não** é
aplicado. Não use `robot_name`; ele existe para o simulador da Booster, que
põe todos os robôs num único grafo DDS.

## 1. Firmware Booster SDK → brain (o emulador publica)

| Tópico | Tipo | Taxa | O que o brain lê | Origem |
|---|---|---|---|---|
| `/low_state` | `booster_interface/LowState` | ≥ 50 Hz | **só** `motor_state_serial[0].q` (yaw da cabeça) e `[1].q` (pitch). O resto pode ir preenchido para o Booster Studio, mas não afeta decisão | `brain.cpp:2130` |
| `/odometer_state` | `booster_interface/Odometer` `{x, y, theta}` | ≥ 50 Hz | pose em frame de odometria. O brain faz `x *= odom_factor` (1.2 no T1) antes de usar | `brain.cpp:2114`, `config.yaml robot.odom_factor` |
| `/head_pose` | `geometry_msgs/Pose` | ≥ 30 Hz | transformação **cabeça → base** (posição + quaternion). Usada para projetar detecções | `brain.cpp:2157` |
| `fall_down_recovery_state` (**relativo**, sem `/`) | `booster_interface/RawBytesMsg` | em mudança + 1 Hz | bytes crus de `{uint8 state; uint8 is_recovery_available}`; `state ∈ {0 READY, 1 FALLING, 2 FALLEN, 3 GETTING_UP}` | `brain.cpp:2182` |
| `/remote_controller_state` | `booster_interface/RemoteControllerState` | em evento | `lt && a` → recalibrar (control_state 2); `lt && b` → entregar ao GC (3); `lt && x` → cancelar (1); `lt && y` → troca de papel; sticks > 0,1 → `go_manual` | `brain.cpp:1623` |
| `/boostercamera/head/rgb/camera_info` | `sensor_msgs/CameraInfo` | latched (QoS transient local) + 1 Hz | `width`, `height` | `brain.cpp:2137`, `config.yaml vision.*` |
| `/boostercamera/head/depth/camera_info` | `sensor_msgs/CameraInfo` | idem | `k[0]` (fx), `k[2]` (cx) para FOV do obstáculo | `brain.cpp:2143` |
| `/boostercamera/head/depth` | `sensor_msgs/Image` (16UC1 ou 32FC1) | opcional, 10–15 Hz | grade de obstáculos (`obstacle_avoidance.*`) | `brain.cpp:258-267` |

Notas:

- `game.xml` faz `RunOnce control_state=3`, então o brain entra em modo
  árbitro **sem** joystick. Ainda assim o emulador expõe a ação "LT+B" pela
  CLI do árbitro, porque o procedimento de pré-jogo (LT+A localizar, LT+B
  entregar) faz parte do que queremos ensaiar.
- Odometria: publicar `verdade / odom_factor` mais deriva (ver §5). Publicar
  a verdade crua faz o brain achar que andou 20 % a mais.
- `LowState` na versão T1 tem 23 motores seriais; a ordem 0 = `AAHead_yaw`,
  1 = `AAHead_pitch` é o único ponto que o brain depende.

## 2. Brain → firmware (o emulador consome)

Tópico `LocoApiTopicReq` (**relativo**, resolve para `/LocoApiTopicReq`),
tipo `booster_msgs/RpcReqMsg {uuid, header, body}`. `header` é JSON
`{"api_id": N}`; `body` é JSON do parâmetro. Origem: `robot_client.cpp`,
`booster_interface/message_utils.cpp`, `booster/robot/b1/b1_loco_api.hpp`.

| `api_id` | Nome (SDK) | `body` | Quem chama no brain | O emulador faz |
|---|---|---|---|---|
| 2000 | `kChangeMode` | `{"mode": 4}` (kSoccer) ou `{"mode": 0}` (kDamping) | `changeRobocupMode()`, `enterDamping()` | soccer: habilita locomoção; damping: robô "amolece" (cai se estiver de pé) |
| 2001 | `kMove` | `{"vx", "vy", "vyaw"}` m/s, rad/s, já limitados por `vx_limit`/`vy_limit`/`vtheta_limit` e com `min_vx`… aplicados | `setVelocity()` | alimenta `locomotion/` |
| 2004 | `kRotateHead` | `{"pitch", "yaw"}` rad | `moveHead()` | alvo de junta da cabeça, com limite de velocidade do servo |
| 2005 | `kWaveHand` | `{"hand_index", "hand_action"}` | `waveHand()` | ignorar (log) |
| 2008 | `kGetUp` | `{}` ou `{"version"}` | `standUp()` | inicia sequência de levantar: `fall_down_recovery_state` → 3, depois 0, com duração do robô real |
| 2038 | `kVisualKick` | `{"start": bool, "version": 1\|2}` | `RLVisionKick()`, `robocupWalk()` (start=false) | no T1 `enableAutoVisualKick: false`; tratar `start=true` como chute (`locomotion/kick.py`) e `start=false` como cancelamento |

Não existem mais os ids `100001`, `100008`, `100011`, `100012` do fork
antigo. `2024 kShoot` não é usado pelo brain do sandbox.

O brain **não** publica `/joint_ctrl`. Manter a assinatura de `LowCmd` é
opcional (útil para o `booster_deploy`), fora do contrato.

Telemetria que o brain publica e o emulador **ignora**: `/kick_ball`,
`/booster_soccer/robot_pose`, `/booster_soccer/ball_position`,
`/booster_soccer/teammates_poses`, `/booster_soccer/field_dimensions`,
`/booster_soccer/log/*`, `/booster_soccer/visualization_*`. Servem para
gravar/observar o jogo (Rerun, Foxglove).

## 3. Visão → brain (o emulador publica)

O `vision_node` real publica com `rclcpp::QoS(1)` (`vision_node.cpp:282,292`).
Manter profundidade 1.

### `/booster_soccer/detection` — `vision_interface/Detections`

| Campo | Uso no brain | Como preencher |
|---|---|---|
| `header.stamp` | latência (`timeLastDet`), idade da bola | wall clock do instante da "captura" |
| `detected_objects[].label` | `"Ball"`, `"Goalpost"`, `"Person"`, `"Opponent"`, `"LCross"`, `"TCross"`, `"XCross"`, `"PenaltyPoint"` (`brain.cpp:2041-2055`) | do ground truth, filtrado por FOV e oclusão |
| `.color` | cor da camisa em `Opponent` | cor do time adversário no `match.yaml` |
| `.confidence` | `ball_confidence_threshold: 50` (escala 0–100) | modelo de erro: cai com distância e borda do FOV |
| `.xmin .ymin .xmax .ymax` | rastreio de câmera (`CamTrackBall`) | projeção pela intrínseca + `head_pose` |
| `.target_uv` (2 floats) | pixel preciso das marcações de chão | só para `*Cross`/`PenaltyPoint` |
| `.position_projection[0:2]` | **posição usada** (x à frente, y à esquerda, m, frame do robô) (`brain.cpp:2463`) | ground truth + ruído ∝ distância |
| `.position` | não usado hoje | igual a `position_projection` |
| `corner_pos` (10 floats) | trapézio de chão visível, 5 cantos × (x, y) no frame do robô (`brain.cpp:2615`) | calcular do FOV atual e pitch da cabeça |
| `radar_x/radar_y` | não usados | vazios |

### `/booster_soccer/line_segments` — `vision_interface/LineSegments`

`coordinates` = `[x0, y0, x1, y1, …]` no frame do robô (m);
`coordinates_uv` = mesmos segmentos em pixel. O `locator` do brain usa isso
para corrigir a odometria (`min_marker_count: 5`, `max_residual: 0.35`).
**Sem linhas o brain nunca localiza**; é obrigatório publicar as linhas do
campo visíveis, recortadas pelo FOV.

### Câmera de referência (T1)

Intrínseca usada hoje em `agent.py`: `fx = fy = 260.66`, `cx = 325.6`,
`cy = 182.0`, imagem 640×380. Confirmar com o `camera_info` do robô real na
primeira gravação (§6) e guardar em `emulator/models/camera.py`.

## 4. Apito → brain

`/whistle_detected`, `std_msgs/String` com `data == "whistle_detected"`
(`whistle_detection/src/main.cpp:31`, `brain.cpp:1750`). O brain, em `SET` e
sendo o lado do kickoff, muda para `PLAY` localmente e **ignora por 12 s** o
GC dizendo `SET` (`brain.cpp:1853`). No lado que não tem o kickoff, espera a
bola sair do círculo central ou timeout.

O emulador recebe do lado do árbitro um evento "apito" com timestamp e, por
robô, aplica atraso (`N(150 ms, 50 ms)`) e probabilidade de perda
(configurável, padrão 5 %). Isso é o que permite testar o `delayAfterPlaying`
de 10 s do GameController de verdade.

## 5. Modelos de erro (o que torna a emulação honesta)

| Modelo | Arquivo | Parâmetros iniciais | Ajustar com |
|---|---|---|---|
| Odometria | `emulator/models/odometry.py` | `scale = 1/odom_factor`, deriva de yaw 0,5°/m, ruído XY 2 % da distância | log do robô real: `/odometer_state` vs pose por mocap/vídeo |
| Visão | `emulator/models/camera.py` | FOV da intrínseca, alcance máx. 8 m, `σ_range = 3 % + 0,05 m`, taxa 30 Hz, latência 60 ms, falso negativo 5 % (+ oclusão por outro robô) | log de `/booster_soccer/detection` no campo com bola parada em distâncias conhecidas |
| Apito | `emulator/models/latency.py` | atraso 150 ± 50 ms, perda 5 % | gravação de `/whistle_detected` em treino |
| Queda | `world/events.py` + `firmware_node` | queda se \|roll\| ou \|pitch\| > 60° por 0,3 s; levantar em 6 s | tempos do `kGetUp` real |

Todos os modelos têm modo `perfect=True` para depuração.

## 6. Marcha e chute: plano em níveis

O firmware é uma política RL fechada; não temos como rodá-la fora do robô.
A emulação é por níveis, cada um substituindo o anterior sem mudar o
contrato acima (`locomotion/base.py`).

| Nível | O que é | Custo | Serve para |
|---|---|---|---|
| **L0** `kinematic.py` | tronco integrado de `vx, vy, vyaw`, juntas fixas, sem queda | pronto | validar estado de jogo, posicionamento, comunicação |
| **L1** `response_model.py` | velocidade real = filtro de 1ª ordem do comando, com atraso puro, aceleração máxima, escorregamento lateral e deriva de yaw, tudo por eixo, ajustado em log real | 1 dia de gravação + ajuste | tempos de chegada, disputa de bola, aproximação de free kick |
| **L2** `learned.py` | RNN/GRU: entrada `(cmd_t, estado_t)` → `(alvos de junta_{t+1}, twist do tronco_{t+1})`; juntas em controle de posição no MuJoCo, contato dos pés/bola pela física | gravações extensas + treino | chute por contato, quedas plausíveis, `LowState` realista |
| **L3** política real | se a Booster liberar `policy.pt` compatível | depende de terceiros | fidelidade máxima |

### O que gravar no robô real para L1/L2

Um `rosbag2` por sessão, a 50–100 Hz, com:

```
/LocoApiTopicReq  /low_state  /odometer_state  /head_pose  fall_down_recovery_state
/boostercamera/head/rgb/camera_info  /boostercamera/head/depth/camera_info
/booster_soccer/detection  /booster_soccer/line_segments  /whistle_detected
/robocup/game_controller  /remote_controller_state
```

mais uma referência externa de pose (vídeo com marcadores no chão ou
mocap), e um roteiro fixo de comandos: degraus de `vx` (0,2 → 1,0 m/s),
`vy` (±0,4), `vyaw` (±1,2), curvas, paradas bruscas, 10 chutes
(`kVisualKick start=true`), 5 quedas induzidas com `kGetUp`. Script em
`tools/record_real_robot.sh` (a criar).

## 7. Sequência de vida de um robô emulado

```
[container sobe]  game_controller_node bind 3838
                  emulator: publica camera_info (latched), /low_state, /odometer_state, /head_pose a 50 Hz,
                            fall_down_recovery_state = READY
                  brain_node: control_state=3 (RunOnce), gc_game_state=INITIAL, aguarda pacote do GC
[GC INITIAL]      brain manda kChangeMode{4}; emulador habilita locomoção (não anda: estado INITIAL)
[GC READY]        brain manda kMove → emulador anda até posição de kickoff
[GC SET]          kMove(0,0,0); cabeça continua
[apito]           referee/whistle → emulador publica /whistle_detected (com atraso/perda)
[GC PLAYING]      até 10 s depois do apito
[queda]           world detecta → emulador: FALLING → FALLEN; brain manda kGetUp → GETTING_UP → READY
[penalidade]      GC marca; brain para; referee/pickup teleporta o robô para fora; ao expirar, ponto de reentrada
[GC FINISHED]     brain para; emulador continua publicando sensores
```
