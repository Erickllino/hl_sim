# Known Bugs — 27/08/2026

Levantamento feito por leitura do código (sem execução). Ordenado por impacto.

> **Atualizado em 27/08/2026** — a migração para o GameController oficial (v20, UDP 3838)
> resolveu os itens 1, 2, 4, 7 e 8. Ver `docs/gamecontroller-integracao.md`.
> Novos achados na seção "Descobertos na migração do GameController".

---

## 1. `main()` do `sim_bridge.py` está quebrado  ✅ RESOLVIDO

**Arquivo:** `bridge/sim_bridge.py:330-341`

Monta 7 agentes (`NUM_ROBOTS = 6` → `ValueError: Scene supports at most 6 robots`) e
chama `ROS2Agent(robot_name="T1_0")` sem o argumento obrigatório `node` → `TypeError`.

`python -m bridge.sim_bridge` / `uv run sim-bridge` não roda. Entrypoint válido hoje é
`scripts/run_ros2.py`.

**Fix aplicado:** `main()` virou execução standalone com 6 `DefaultAgent`, sem importar
`ROS2Agent`. O `SimBridge` não depende mais de ROS2 — importante para treino RL headless.
Flag `--force-playing` para mover os robôs sem GameController.

---

## 2. Os 5 `DefaultAgent` nunca se movem  ✅ RESOLVIDO

**Arquivo:** `bridge/default_agent.py:44,50,112`

`__init__` e `reset()` colocam a fase em `_Phase.STOP`, e nenhum ramo do `step()` sai de
`STOP` — a máquina `SEEK → ALIGN → APPROACH → KICK` nunca inicia. Os robôs scriptados
ficam parados o jogo inteiro.

**Fix aplicado:** a transição `STOP → SEEK` agora depende de `is_active()`, ou seja
`STATE_PLAYING` + não parado + não punido, vindo do GameController real. Sem árbitro os
robôs ficam parados — que é o comportamento **correto** de um robô em competição.

---

## 3. `TRUNK_HEIGHT` não bate com o keyframe

**Arquivos:** `bridge/sim_bridge.py:58` (`TRUNK_HEIGHT = 0.75`) vs
`scenes/soccer_scene.xml` keyframe `home` (z = 0.679)

No primeiro control tick do modo cinemático todos os robôs sobem 7 cm de golpe.

**Fix:** usar 0.679, ou ler a altura inicial do keyframe no `reset()` e guardar por robô.

---

## 4. `print` de debug a ~300 linhas/s  ✅ RESOLVIDO

**Arquivo:** `bridge/sim_bridge.py:176`

`print(f"Agent {i} joint_pos: {cmd.joint_pos}")` roda por agente por control tick
(6 × 50 Hz). Polui o terminal e custa tempo de loop.

**Fix aplicado:** `print` removido.

---

## 5. `ROS2Agent` só funciona no robô de índice 0

**Arquivo:** `bridge/ros2_agent.py:55-60, 96-106`

Dois problemas que bloqueiam a Fase 6 (multi-brain):

- `JOINT_QPOS_START = 14` / `JOINT_QVEL_START = 12` são hardcoded para o primeiro robô.
  Para o robô `i` seria `7 + i*30 + 7` e `6 + i*29 + 6`.
- Todos os publishers/subscribers usam tópicos globais, sem namespace. Duas instâncias
  colidiriam em `/low_state`, `/odometer_state`, `LocoApiTopicReq`, etc.

**Fix:** receber o índice do robô no construtor e derivar os offsets; prefixar tópicos
com `/{robot_name}` (ou usar namespace do nó).

---

## 6. `scripts/test_scene.py` está desatualizado

**Arquivo:** `scripts/test_scene.py:70`

Espera os números de 1 robô (`nq=37, nv=35, nu=23, nbody=28`). A cena 3v3 atual tem
`nq=187, nv=180, nu=138, nbody=148`. Os testes 1–4 falham mesmo com a cena correta.

**Fix:** parametrizar por `NUM_ROBOTS` (`nq = 7 + n*30`, `nv = 6 + n*29`, `nu = n*23`,
`nbody = 4 + n*24`).

---

## 7. `agent_interface.py` importa ROS2 no topo do módulo  ✅ RESOLVIDO

**Arquivo:** `bridge/agent_interface.py:7`

`from game_controller_interface.msg import GameControlData` no nível do módulo faz o
`DefaultAgent` (que é Python puro) exigir o workspace ROS2 buildado. Quebra a promessa de
"agente standalone sem dependência externa".

**Fix aplicado:** o `_publish_game_controller()` inteiro deixou de existir — o sim não
produz mais estado de jogo. `agent_interface.py` agora é Python puro (só `numpy`), e o
consumo do tópico ficou isolado em `bridge/game_controller_link.py`.

---

## 8. Nomes dos agentes não batem com os robôs da cena  ✅ RESOLVIDO

**Arquivo:** `scripts/run_ros2.py:26-33`

A ordem dos `<include>` na cena é **1, 3, 5, 2, 4, 6**, então o agente de índice 1
(`"T1_1"`) na verdade controla o `robot3`, o índice 2 (`"T1_2"`) controla o `robot5`, etc.
Cosmético, mas confunde na hora de depurar.

**Fix aplicado:** renomeados para `T1_1, T1_3, T1_5, T1_2, T1_4, T1_6`, na ordem real da
cena, com `team_id`/`player_id` explícitos por agente.

---

## Fragilidade estrutural (não é bug, mas vale registrar)

Os endereços de `qpos`/`qvel`/`body_id`/`sensordata` em `bridge/sim_bridge.py:60-80` são
calculados por aritmética de offset em vez de `mj_name2id` / `model.body()`. Qualquer
corpo novo na cena (uma trave extra, um marcador) desloca tudo **silenciosamente** — sem
erro, só comportamento errado.

**Sugestão:** resolver os IDs por nome no `__init__` do `SimBridge`.

---
---

# Descobertos na migração do GameController (27/08/2026)

## 9. `use_sim_time` sem `/clock` (prioridade alta)

`brain/launch/launch.py:44` seta `use_sim_time: True` quando se passa `sim:=true`, mas o
sim **nunca publica `/clock`**. O relógio do brain congela em t=0 e todo cálculo de
`now() - stamp` vira lixo.

O curativo atual é `ros2_agent.py` carimbar as detecções com **wall clock**
(`_publish_detections`). Funciona por coincidência a `--speed 1.0` e **quebra** a
`--speed 2.0` ou `--speed 0`.

**Fix (curto prazo):** rodar o brain com `sim:=false`.
**Fix (correto):** o sim publicar `rosgraph_msgs/Clock` com `d.time` a cada control tick.
Destrava rodar mais rápido que tempo real, que é pré-requisito para treino RL.

---

## 10. `vision_node` e `game_controller_node` competindo com o sim

`hsl-player/scripts/sim_start.sh` sobe `vision_node` **e** `game_controller`. O
`vision_node` publica `/booster_vision/detection` a partir da câmera ZED, o mesmo tópico
que o sim publica com ground truth → dois publishers brigando.

**Para simulação:** rodar `game_controller` (ele é a ponte do árbitro, agora obrigatória)
mas **nunca** o `vision_node`. Vale um `sim_start.sh` próprio no hl_sim.

---

## 11. Colisão de portas UDP entre brains do mesmo time

`brain_communication.cpp:24-25`:
```cpp
_discovery_udp_port = 20000 + teamId;   // broadcast
_unicast_udp_port   = 30000 + teamId;   // unicast entre companheiros
```

As portas dependem **só do `teamId`**. Vários brains do mesmo time no mesmo host bindam a
mesma porta. O discovery se protege filtrando `msg.playerId` (`:283`), mas o unicast
(`:387`) endereça companheiros por `IP:porta` — e no localhost todos são
`127.0.0.1:30000+teamId`. Mensagem de time cai no socket errado.

Namespace ROS2 **não** resolve: isso é socket cru, fora do DDS.

**Fix (curto prazo):** rodar com `disable_com:=true` em 3v3 local.
**Fix (correto):** um container por robô com IP próprio, ou offset de porta por playerId
(exigiria mudar o brain).

---

## 12. Versão do struct do GameController 2026  ✅ VERIFICADO — compatível

**Fonte:** https://github.com/RoboCup-HumanoidSoccerLeague/GameController
(`game_controller_msgs/headers/RoboCupGameControlData.h`)

O header do GameController oficial é **idêntico** ao de
`hsl-player/src/game_controller/include/RoboCupGameControlData.h`:

| | Oficial (HL, Rust) | hsl-player |
|---|---|---|
| `GAMECONTROLLER_STRUCT_HEADER` | `"RGme"` | `"RGme"` |
| `GAMECONTROLLER_STRUCT_VERSION` | `20` | `20` |
| `GAMECONTROLLER_RETURN_STRUCT_HEADER` | `"RGrt"` | `"RGrt"` |
| `GAMECONTROLLER_RETURN_STRUCT_VERSION` | `4` | `4` |
| `MAX_NUM_PLAYERS` | `20` | `20` |
| portas | 3838 / 3939 | 3838 / 3939 |

Ordem e tipos dos campos de `RobotInfo`, `TeamInfo`, `RoboCupGameControlData` e
`RoboCupGameControlReturnData` conferem. Pacote de 158 bytes, aceito pelo parser.

`HL_MAX_NUM_PLAYERS 11` é adição do hsl-player, fora do protocolo — só dimensiona
arrays internos do brain.

**Ainda vale rodar** `python3 scripts/gc_sniffer.py` na primeira subida: confirma que o
broadcast está chegando pela interface certa, que é onde o problema costuma estar agora
que o formato está descartado como suspeito.

---

## 13. `team_id` do sim divergia do brain

`run_ros2.py` passava `team_id=42` enquanto `config.yaml:4` do brain tem `team_id: 56`.
O `brain.cpp:1407` descarta o pacote de GC inteiro quando nenhum time bate — ou seja, o
brain **nunca** aceitou um pacote de estado de jogo do sim.

**Fix aplicado:** `--team-id` (default 56) e `--oppo-team-id` (default 57) em
`run_ros2.py`, com o valor propagado para todos os agentes. Os três números —
sim, `config.yaml` do brain e cadastro no GameController — precisam ser o mesmo.

---

## 14. Canal de mensagens de time fora do padrão do GameController

O GameController oficial escuta mensagens de time em **`10000 + team number`** e
contabiliza o `messageBudget` (campo em `TeamInfo`). O `brain_communication.cpp` usa
portas próprias (`20000 + teamId` broadcast, `30000 + teamId` unicast) e **não** passa
pelo canal padrão.

Consequência: o árbitro não enxerga o tráfego de time do hsl-player. Em simulação isso é
inofensivo; em competição, confirmar se a HL fiscaliza o message budget nesta temporada.

Relacionado ao item 11 (colisão de portas entre brains do mesmo host).
