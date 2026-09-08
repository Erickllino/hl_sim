# 01 — Arquitetura-alvo do `hl_sim`

> Documento normativo para a refatoração. Descreve **onde queremos chegar**, não o
> código de hoje. O caminho está em `04-plano-refatoracao.md`.

## 1. Princípios

1. **O container do robô é o robô.** Roda o `robocup_demo` (branch
   `sandbox/support_2026_game_controller`) **sem nenhum patch**, com os mesmos
   launch files, o mesmo `config.yaml`, o mesmo `fastdds.xml` e o mesmo caminho
   de workspace. Se um dia precisarmos mexer no brain, a mudança vai para o
   repositório do brain, nunca para o `hl_sim`.
2. **O `hl_sim` só substitui o que é hardware ou mundo.** Firmware do Booster
   SDK (locomoção, odometria, IMU, joystick, estado de queda), câmera + nó de
   visão, microfone + detector de apito, o campo, a bola, os adversários e o
   Wi-Fi. Nada além disso.
3. **O que atravessa a fronteira do robô é só o que atravessa na competição.**
   UDP 3838 (árbitro → robôs), UDP 3939 (robô → árbitro), UDP `10000+time`
   (robô ↔ robô) e, opcionalmente, 1 pacote/s de debug para uma máquina do
   time. DDS **não sai** do container, exatamente como não sai do Jetson.
4. **Estado de jogo vem do GameController oficial.** O sim nunca publica
   `/robocup/game_controller`. Para rodar sem o app (CI, treino), existe um
   emissor de pacotes `RGme` v20 **wire-idêntico** na porta 3838, e o
   `game_controller_node` real continua sendo quem faz o parse.
5. **Uma fonte de configuração.** `config/match.yaml` gera tudo que precisa
   bater: `team_id`/`player_id` do brain, IP e whitelist do árbitro, domínio de
   rede, mapeamento container → corpo na cena.
6. **Relógio de parede, tempo real.** O robô real não tem `/clock`. Em modo de
   fidelidade o mundo roda a 1× e os stamps são `time.time()`. Rodar mais
   rápido que tempo real é um modo separado, sem brains, para RL.
7. **Sem ROS2 fora do emulador.** `world/`, `locomotion/`, `agents/`,
   `protocol/` são Python puro (numpy + mujoco), importáveis para treino
   headless sem workspace ROS2.

## 2. Fronteira de fidelidade

| Componente | Na competição | No `hl_sim` alvo | Fidelidade |
|---|---|---|---|
| `brain_node` | Booster sandbox | **idêntico** (mesmo binário, mesmo config) | bit a bit |
| `game_controller_node` | Booster sandbox | **idêntico** | bit a bit |
| GameController | app oficial no PC do árbitro | app oficial no host (ou emissor wire-idêntico em CI) | protocolo |
| Mensagens de time | UDP `10000+time` broadcast entre robôs via AP | UDP `10000+time` broadcast na bridge Docker | protocolo |
| Retorno ao árbitro | `RGrt` v4 unicast 3939 | idem, chega no host | protocolo |
| Firmware Booster SDK | processo no Jetson, DDS local `rt/*` | `emulator/firmware_node` no container do robô | contrato ROS2 |
| `vision_node` | YOLO na câmera ZED / D-Robotics | `emulator/vision_node` a partir do ground truth com FOV, ruído e taxa de câmera | contrato ROS2 + modelo de erro |
| `whistle_detection` | ALSA + FFTW no microfone | `emulator/whistle_node`, disparado pelo lado do árbitro com atraso e taxa de perda | contrato ROS2 + modelo de erro |
| Joystick (LT+A / LT+B) | handler com controle físico | `emulator/firmware_node` publica `/remote_controller_state` sob comando do operador | contrato ROS2 |
| Marcha e chute | política RL no firmware | `locomotion/` (cinemático → modelo aprendido no robô real) | modelo |
| Campo, bola, quedas | físico | MuJoCo | modelo |
| Wi-Fi | AP compartilhado, perda e jitter | bridge Docker, com perfil `netem` opcional | modelo |
| Árbitro humano | apita, posiciona bola, retira robô punido | operador via CLI `referee` (mesmas ações, mesmos tempos) | procedimento |

O que fica **fora** de propósito: `vision_node` real (precisa de CUDA/ONNX e
câmera), `whistle_detection` real (precisa de ALSA), modo agente
(`/booster_agent/*`), Booster Studio.

## 3. Camadas e processos

```
HOST
┌──────────────────────────────────────────────────────────────────────────────────┐
│ GameController oficial (Tauri)  · interface br-hl_net (172.28.0.1)               │
│   ▲ UDP 3939 RGrt (por robô)    │ UDP 3838 RGme broadcast   ▲ UDP 10000+time     │
│   │                             ▼                           │ (contagem)         │
│ referee CLI: whistle · ball · pickup · sniffer · team-monitor · auto-gc (CI)      │
└──────────────┬───────────────────────────────────────────────────────────────────┘
               │ link (ZMQ/msgpack, 50 Hz por robô)            bridge hl_net 172.28.0.0/16
┌──────────────┴───────────┐        ┌───────── container robotN · IP 172.28.0.1N ──────────┐
│ container `world`        │◀──────▶│  DDS só em 127.0.0.1 (fastdds.xml do robocup_demo)    │
│ MuJoCo: campo, bola,     │        │  ┌───────────────┐  /robocup/game_controller          │
│ 6 corpos T1, contatos,   │        │  │game_controller│──────────────┐                     │
│ locomoção, eventos       │        │  │_node (UDP3838)│              ▼                     │
│ (gol, fora, queda)       │        │  └───────────────┘        ┌──────────┐  LocoApiTopicReq│
│ headless ou viewer       │        │  ┌───────────────┐        │brain_node│─────────┐       │
│ agentes scriptados       │        │  │emulator:      │◀───────┤(Booster) │         │       │
│ (adversário sem brain)   │        │  │ firmware_node │ /low_state /odometer_state  │       │
└──────────────────────────┘        │  │ vision_node   │ /booster_soccer/detection   ▼       │
                                    │  │ whistle_node  │ /whistle_detected     RGrt → host   │
                                    │  └───────────────┘                                     │
                                    └─────────────────────────────────────────────────────────┘
```

- **`world`** (1 processo). Dono da física e da verdade. Não conhece ROS2.
  Expõe, pelo `link`, o estado de cada robô (pose, juntas, IMU, contato com o
  chão, caiu/não caiu) e do mundo (bola, outros robôs) e recebe comandos
  (velocidade, cabeça, chute, levantar, teleporte do árbitro).
- **`emulator`** (1 por robô, **dentro** do container do robô). Único lugar
  com `rclpy`. Traduz `link` ↔ tópicos ROS2 com os nomes, tipos, QoS e taxas
  do firmware e da visão. Aplica os modelos de erro (odometria, visão, apito).
- **`referee`** (host). Ferramentas do lado do árbitro humano e do PC do
  árbitro. Nunca fala ROS2; fala UDP (3838/3939/10000+time) e `link`.
- **`agents`** (dentro do `world`). Adversários scriptados que obedecem ao
  árbitro pelo mesmo pacote UDP que os brains recebem (via `protocol/`).

Por que o emulador roda no container do robô e não no `world`: é o que faz o
DDS ficar confinado ao loopback, como no Jetson, e permite usar o
`configs/fastdds.xml` do `robocup_demo` sem edição. O `ROS_DOMAIN_ID` por robô
de hoje deixa de ser necessário (cada container tem sua própria rede e seu
próprio grafo DDS em `127.0.0.1`).

## 4. Árvore de diretórios alvo

```
hl_sim/
├── config/
│   ├── match.yaml                # ÚNICA fonte: times, jogadores, papéis, IPs, árbitro
│   └── generated/                # gitignored: config_local.yaml por robô, params do GC node, .env
├── assets/  scenes/              # MuJoCo (campo 14×9, bola, robotN/*.xml) — como hoje
├── src/hl_sim/
│   ├── world/                    # Python puro. MuJoCo, sem ROS.
│   │   ├── model.py              #   carga da cena, resolução de índices por nome (layout.py de hoje)
│   │   ├── physics.py            #   loop 200 Hz / controle 50 Hz, aplica comandos, lê sensores
│   │   ├── events.py             #   gol, bola fora, robô fora, queda, "ball free" — só LOGA/avisa; não muda estado de jogo
│   │   ├── referee_actions.py    #   teleporte de robô punido/reentrada, posicionamento de bola, kickoff
│   │   └── server.py             #   expõe o link para N emuladores + agentes scriptados
│   ├── locomotion/               # como um comando vira movimento (plugável)
│   │   ├── base.py               #   interface: step(cmd, state) -> alvos de junta / twist do tronco
│   │   ├── kinematic.py          #   o de hoje (tronco integrado, juntas fixas)
│   │   ├── response_model.py     #   L1: dinâmica de 1ª ordem + atraso + limites ajustados em log real
│   │   ├── learned.py            #   L2: RNN treinada em log real → trajetória de juntas + twist
│   │   └── kick.py               #   modelo de chute (contato / impulso / trajetória)
│   ├── link/                     # protocolo world ↔ emulador
│   │   ├── schema.py             #   dataclasses versionadas: RobotState, WorldState, RobotCmd, RefereeCmd
│   │   └── transport.py          #   ZMQ + msgpack; um socket por robô; 50 Hz
│   ├── emulator/                 # ÚNICO pacote com rclpy. Roda no container do robô.
│   │   ├── firmware_node.py      #   /low_state /odometer_state /head_pose fall_down_recovery_state
│   │   │                         #   /remote_controller_state ; assina LocoApiTopicReq (/joint_ctrl opcional)
│   │   ├── vision_node.py        #   /booster_soccer/detection /booster_soccer/line_segments + camera_info
│   │   ├── whistle_node.py       #   /whistle_detected
│   │   ├── models/               #   odometry.py (fator+deriva), camera.py (FOV, projeção, ruído), latency.py
│   │   └── launch/emulator.launch.py
│   ├── protocol/                 # Python puro
│   │   ├── gamecontroller.py     #   RGme v20 (parse+build), RGrt v4 (parse+build), tabelas v20→v19
│   │   └── team_message.py       #   TeamCommunicationMsg do Booster (para monitor e agentes)
│   ├── agents/                   # scripted.py — adversário sem ROS, obedece ao GC via protocol/
│   ├── referee/                  # lado do árbitro, host, sem ROS
│   │   ├── whistle.py            #   "apita": manda evento a todos os emuladores (atraso/perda por robô)
│   │   ├── ball.py               #   posiciona bola (free kick, penalty, kickoff, drop ball)
│   │   ├── pickup.py             #   retira robô punido / recoloca no ponto de reentrada
│   │   ├── gc_sniffer.py         #   o de hoje
│   │   ├── gc_relay.py           #   o de hoje (plano B)
│   │   ├── team_monitor.py       #   escuta 10000+time, conta e mede como o GC oficial
│   │   └── auto_gc.py            #   emissor RGme v20 wire-idêntico para CI/treino (sem UI)
│   ├── config.py                 # carrega match.yaml
│   └── cli/                      # run_world, run_emulator, run_match, gen_configs, referee
├── docker/
│   ├── Dockerfile                # targets: robot (robocup_demo sandbox + SDK), world, emulator
│   ├── compose.yaml              # world + robot1..6 (cada um sobe brain+gc+emulator), perfis relay/netem
│   ├── launch/robot.launch.py    # = start.sh do robocup_demo sem vision/whistle + emulator.launch.py
│   └── fastdds/                  # NENHUM perfil próprio: usa configs/fastdds.xml do robocup_demo
├── tools/                        # scripts auxiliares (record_real_robot.sh, plot_gait.py)
├── tests/
│   ├── test_protocol_gc.py       # round-trip RGme/RGrt contra o header C oficial
│   ├── test_msg_contract.py      # campos/tipos das msgs == pacotes de interface clonados
│   └── scenarios/                # kickoff, brief_stop, penalty, free_kick, budget, gc_loss
└── docs/
```

Regras da árvore:

- `world/`, `locomotion/`, `link/`, `protocol/`, `agents/`, `referee/` **não
  importam `rclpy`**. Um teste garante isso.
- Só `emulator/` conhece nomes de tópico ROS2. Se a Booster renomear um
  tópico, muda um arquivo.
- `protocol/gamecontroller.py` é a única definição do struct do árbitro e é
  testada contra `RoboCupGameControlData.h` clonado do repositório oficial.

## 5. Containers e imagens

| Imagem | Conteúdo | Origem | Observação |
|---|---|---|---|
| `hl-robot` | ROS2 Humble, `robocup_demo` sandbox (`brain`, `game_controller`, `interface/*`), headers + lib do `booster_robotics_sdk`, `hl_sim.emulator` | clone **HTTPS público** (sem chave SSH) | workspace em `/home/booster/Workspace/robocup_demo`, usuário `booster`, `FASTRTPS_DEFAULT_PROFILES_FILE` apontando para o `configs/fastdds.xml` **do próprio repo** |
| `hl-world` | MuJoCo, `hl_sim.world` + `locomotion` + `agents` + `link` | este repo | sem ROS2 |
| `hl-referee` | `hl_sim.referee` + `protocol` | este repo | `network_mode: host` |

Cada `robotN` sobe, nessa ordem: `game_controller_node` (bind 3838),
`emulator` (firmware + vision + whistle), `brain_node`. É o `start.sh` da
Booster sem `vision launch.py` e sem `whistle_detection`.

O brain lê `config.yaml` → `config_local.yaml` → `~/agents/booster_soccer/brain.yaml`.
Só o `config_local.yaml` é nosso, gerado de `match.yaml`, e contém apenas o que
difere por robô ou por ambiente (ver `03-rede-e-gamecontroller.md`, §5).

## 6. Modos de execução

| Modo | O que sobe | Relógio | Para quê |
|---|---|---|---|
| `match` | world + N robôs com brain + GC oficial no host | parede, 1× | validar estratégia; é o modo de fidelidade |
| `match --auto-gc` | idem, mas `referee/auto_gc.py` no lugar do app | parede, 1× | CI, testes de cenário sem operador |
| `scrimmage` | world + agentes scriptados + auto-gc | livre (`--speed 0`) | treino RL, regressão de física |
| `bench` | um robô com brain, sem GC | parede | depurar emulador |

Robôs não listados em `--brains` recebem `agents.ScriptedAgent` no `world`;
a cena tem sempre os 6 corpos.

## 7. Decisões registradas

- **Link world ↔ emulador: ZeroMQ + msgpack.** Python puro, sem IDL, baixa
  latência local, funciona entre containers pela bridge. Alternativa
  descartada: usar DDS para isso (vazaria DDS para fora do robô e misturaria
  domínios).
- **`ROS_DOMAIN_ID` por robô deixa de existir.** Isolamento passa a ser por
  rede (namespace do container) e pelo `interfaceWhiteList=127.0.0.1` do
  `fastdds.xml` oficial. Enquanto o emulador ainda rodar no `world` (fase
  intermediária), o domínio por robô continua sendo o mecanismo.
- **Odometria emulada é a do firmware, não a verdade.** O brain multiplica por
  `odom_factor` (1.2 no T1); o emulador tem que publicar um valor que, após o
  fator, se aproxime da verdade com a deriva do robô real.
- **Eventos do mundo não mudam o estado de jogo.** Gol, bola fora e robô fora
  geram aviso para o operador do GC, como na competição. Um "auto-árbitro"
  que aciona o `auto_gc` existe só no modo `scrimmage`.
