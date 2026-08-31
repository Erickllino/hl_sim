# Integração com o GameController oficial

Decisão de arquitetura (27/08/2026): o simulador **não produz** estado de jogo.
Ele consome o mesmo tópico que o brain consome, alimentado pelo GameController real.

## Por quê

O objetivo do `hl_sim` é chegar o mais perto possível da situação de competição.
Um mock ROS2 pulava três coisas que quebram em campo:

1. o parsing do pacote binário `RGme` v20 (tamanho, versão, endianness)
2. o `game_controller_node` do hsl-player, que é quem realmente roda no robô
3. o canal de retorno robô → árbitro (`RGrt` v4, UDP 3939)

## Topologia

Fonte oficial: https://github.com/RoboCup-HumanoidSoccerLeague/GameController (Rust).
Protocolo verificado idêntico ao do hsl-player — ver item 12 em `27082026-knownbugs.md`.

Outras portas do app, hoje não usadas pelo sim: 3636 (monitor requests),
3940 (status forwarding), 10000+team (mensagens de time).

```
        ┌──────────────────────────┐
        │ GameController oficial   │  (app 2026, Rust)
        └───────────┬──────────────┘
        UDP 3838    │            ▲  UDP 3939 (RGrt v4)
        broadcast   ▼            │  um por robô, enviado pelo brain
        ┌──────────────────────────┐
        │ game_controller_node     │  hsl-player, C++
        └───────────┬──────────────┘
                    │ /robocup/game_controller
        ┌───────────┴───────────────┐
        ▼                           ▼
  ┌───────────┐            ┌──────────────────────┐
  │  brain    │            │ GameControllerLink   │  hl_sim
  └─────┬─────┘            └──────────┬───────────┘
        │ LocoApiTopicReq             │ GameState
        ▼                             ▼
  ┌──────────────────────────────────────────────┐
  │           SimBridge  /  MuJoCo               │
  └──────────────────────────────────────────────┘
        │ /low_state /odometer_state /booster_vision/*
        └────────────────▶ de volta ao brain
```

O retorno 3939 sai do `brain_communication.cpp` sem passar pelo sim — é o brain
reportando ao árbitro sua pose, se caiu e onde vê a bola. Já funciona de graça.

## Ordem de subida

```bash
# 1. GameController oficial (host) — broadcast na 3838
#    Confirme com o sniffer, em outro terminal:
python3 scripts/gc_sniffer.py

# 2. ponte UDP → ROS2 (workspace do hsl-player)
cd ~/Documents/hsl-player && ./scripts/start_game_controller.sh

# 3. brain
cd ~/Documents/hsl-player && ./scripts/start_brain.sh sim:=false

# 4. simulador
cd ~/Documents/hl_sim && ./scripts/run_ros2.sh --viewer
```

`sim:=false` no passo 3 é proposital: `sim:=true` liga `use_sim_time` e o sim ainda
não publica `/clock` (issue #9 em `27082026-knownbugs.md`).

**Não suba o `vision_node`** — ele disputa `/booster_vision/detection` com o sim.

## Os três números que precisam ser iguais

| Onde | Campo | Valor atual |
|---|---|---|
| GameController | número do time cadastrado | (definido no app) |
| `hsl-player/src/brain/config/config.yaml` | `game.team_id` | 56 |
| `hl_sim/scripts/run_ros2.py` | `--team-id` | 56 |

O mesmo vale para `game.player_id` (3) e `--player-id`. Se divergirem, o
`brain.cpp:1407` descarta o pacote inteiro e o robô fica em INITIAL para sempre —
sem erro visível além de uma linha de log.

## Diagnóstico

```bash
# o árbitro está emitindo? o pacote é aceitável?
python3 scripts/gc_sniffer.py

# a ponte está publicando?
# ATENÇÃO: o GameController emite a ~2 Hz (5 Hz com o jogo parado), não a 50 Hz.
ros2 topic hz /robocup/game_controller
ros2 topic echo /robocup/game_controller --once

# o retorno dos robôs está saindo?
sudo tcpdump -i any -X udp port 3939
```

O sniffer avisa explicitamente quando o pacote seria descartado pelo
`game_controller_node` (tamanho ≠ 158 bytes, header ≠ `RGme`, version ≠ 20).

## O que ainda falta para fidelidade plena

- **READY**: os robôs deveriam caminhar até a posição de kickoff. Hoje ficam parados.
- **SET play**: `set_play` e `kicking_team` chegam ao `GameState` mas os agentes
  scriptados ainda os ignoram.
- **Reposicionamento**: com o GameController oficial, gol e falta são marcados por um
  operador humano; o sim não reposiciona bola nem robôs automaticamente.
- **`/clock`**: sem ele o sim não roda mais rápido que tempo real com o brain junto.
