




hl_sim/
├── config/
│   ├── match.yaml                # ÚNICA fonte: times, jogadores, papéis, IPs, árbitro
│   └── generated/                # gitignored: config_local.yaml por robô, params do GC node, .env
├──             # MuJoCo (campo 14×9, bola, robotN/*.xml) — como hoje
├── src/
│   assets/  
│   scenes/      
│   hl_sim/
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
│   │   ├── 
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




HOST (notebook)                                   Docker bridge hl_net 172.28.0.0/16
┌────────────────────────┐   UDP 3838 broadcast   ┌──────────── container robotN (IP .1N, ROS_DOMAIN_ID=N) ────────────┐
│ GameController oficial │ ─────────────────────▶ │ game_controller_node ──/robocup/game_controller──▶ brain_node       │
│ (app Tauri, Rust)      │  (ou via gc_relay,     │        │                                            │  ▲            │
│                        │   network_mode: host)  │        │ mesmo domínio DDS                          │  │            │
└────────────────────────┘ ◀──── UDP 3939 RGrt ── │        ▼                                            ▼  │            │
                                                  │ [GameControllerLink]                     LocoApiTopicReq │ /low_state │
                                                  └────────┬───────────────────────────────────────┬────────┴─/odometer──┘
                                                           ▼                                       ▼          /booster_vision/*
                                                  ┌──────────── container sim (IP .10) ───────────────────────────────────┐
                                                  │ RobotLink ×N (um rclpy.Context por domínio)  →  SimBridge → MuJoCo      │
                                                  │ 50 Hz controle / 200 Hz física · locomoção cinemática (tronco teleportado)│
                                                  └───────────────────────────────────────────────────────────────────────┘