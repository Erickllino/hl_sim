# 03 — Rede e GameController como na competição

> Como reproduzir, em Docker, o que as regras (`HSL-Rules`, draft 2026) e o
> GameController oficial impõem na rede do jogo. Valores numéricos vêm de
> `HSL-Rules/common/variables.tex`, do `README.md` do GameController e de
> `config/*/params.yaml` do GameController.

## 1. O que as regras exigem (`rules/robot_players.tex` §Communication)

| Regra | Valor | Consequência para o `hl_sim` |
|---|---|---|
| Só há comunicação robô↔robô, robô↔árbitro, robô↔GC | — | nenhum outro fluxo pode sair do container do robô |
| Wi-Fi do organizador, faixa de IP por time, ad-hoc proibido | anunciado no local | IPs dos containers vêm de `match.yaml`; nada hardcoded no brain |
| Mensagens de time: UDP **broadcast**, porta `10000 + número do time`, payload ≤ 512 B, **unicast proibido** | — | brain do sandbox já cumpre (`brain_communication.cpp`); `team_monitor` verifica |
| Orçamento por jogo | 12.000 msgs; +600 por minuto de acréscimo; +6.000 na prorrogação | 3 robôs × 1 Hz × 20 min = 3.600. Manter `team_comm_frequency_hz ≤ 2` |
| Contagem só em `ready`/`set`/`playing` | — | `team_monitor` replica |
| Violação de tamanho ou orçamento | placar zerado, gols anulados | é a falha mais cara: cenário de teste obrigatório |
| GC → robôs | 2 pacotes/s (5/s em stop) | GC oficial faz; `auto_gc` replica |
| Robôs → GC | obrigatório; 0,5–2 Hz | brain manda 2 Hz para `game_control_ip` |
| Debug | 1 pacote UDP/s por robô para **uma** máquina do time, rede cabeada | opcional; se usar, é o único tráfego extra permitido |
| Apito → playing | GC atrasa 10 s (`delayAfterPlaying`) | `referee/whistle` dispara antes do operador clicar |
| Brief stop | flag `stopped`, parar imediatamente, nem levantar | brain trata; agentes scriptados também (`GameState.is_playing()`) |
| READY | 45 s para chegar em posição legal | cronômetro do próprio GC |

## 2. Portas e sentidos

```
                  PC do árbitro (GameController)                       host, iface br-hl_net 172.28.0.1
       ┌─────────────────────────────────────────┐
 3838  │  RGme v20, 158 B, broadcast, 2 Hz (5 Hz stop) ───────────────▶ todos os robôs (game_controller_node)
 3939  │  ◀── RGrt v4, 32 B, unicast, 2 Hz por robô ──────────────────── brain (game_control_ip = 172.28.0.1)
10000+T│  escuta broadcast do time T, conta e mede ◀────────────────────── brain ↔ brain (TeamCommunicationMsg ≤ 512 B)
 3636  │  ◀── monitor request "RGTr"  (ferramentas de monitor; robôs NÃO)
 3838  │  RGTD unicast para monitores (estado verdadeiro, sem o "estado falso")
 3940  │  status forwarding para monitores
       └─────────────────────────────────────────┘
```

Detalhes que enganam:

- **Estado falso.** Após uma transição para `playing`, o GC continua enviando
  o estado anterior por até 10 s (`delayAfterPlaying`) ou até outro evento.
  Um monitor (3636) recebe o estado verdadeiro, um robô não. Quem já mandou
  `RGrt` nunca vira monitor, e vice-versa. O `gc_sniffer` roda como robô
  (recebe o broadcast), então mostra o que o robô vê.
- **Whitelist do nó.** `game_controller/launch/launch.py` do sandbox vem com
  `enable_ip_white_list: True` e `ip_white_list: ["192.168.30.170"]`. Pacote
  de outro IP é descartado com um `RCLCPP_INFO`. Em sim o IP tem que ser
  `172.28.0.1` (host) ou o IP de origem do `gc_relay`; na competição, o IP
  do PC do árbitro anunciado no local. Gerado por `match.yaml`.
- **`game_control_ip`.** Parâmetro do brain (`config.yaml`, padrão
  `192.168.30.170`). Se estiver errado, o retorno 3939 vai para o nada e o
  árbitro não vê o robô. Gerado por `match.yaml`.
- **Penalidades chegam em numeração v19.** O nó normaliza v20 → v19 antes de
  publicar (`game_controller_node.cpp normalize_v20_penalty`): `SENT_OFF=10`,
  `SUBSTITUTE=11`, `MOTION_IN_STOP` vira `MOTION_IN_SET`, `CAUTIONED` vira
  `PUSHING`. `protocol/gamecontroller.py` tem que ter as duas tabelas: v20
  para o fio, v19 para o tópico.

## 3. Topologia Docker alvo

```
host
├── br-hl_net (172.28.0.1/16)   ← GameController oficial ligado NESTA interface
│   ├── hl-world   172.28.0.10  (sem ROS; fala link com os emuladores)
│   ├── hl-robot1  172.28.0.11  time casa
│   ├── hl-robot3  172.28.0.13
│   ├── hl-robot5  172.28.0.15
│   ├── hl-robot2  172.28.0.12  time visitante
│   ├── hl-robot4  172.28.0.14
│   └── hl-robot6  172.28.0.16
└── hl-referee (network_mode: host)  ← whistle, ball, pickup, sniffer, team_monitor, auto_gc
```

- **GameController no host, bound em `br-hl_net`.** O app escolhe a interface
  no launcher. Nessa interface o broadcast 3838 chega aos containers sem
  relay, o `RGrt` volta para `172.28.0.1`, e as mensagens de time em
  `10000+T` (broadcast limitado dentro da bridge) chegam ao host e **são
  contadas**. O `gc_relay` fica como plano B, para quando o app insiste na
  interface da rota padrão.
- **Dentro do container do robô, DDS só em loopback.** Usar
  `configs/fastdds.xml` do `robocup_demo` como está (`interfaceWhiteList`
  `127.0.0.1`, `192.168.10.101/102`). Os dois IPs internos do robô não
  existem no container e são ignorados. Resultado: `tcpdump -i eth0 udp
  portrange 7400-7500` no container tem que ficar **vazio**.
- **Um robô = um namespace de rede.** Sockets do brain (`bind 0.0.0.0:3838`
  sem `SO_REUSEADDR`, `bind 0.0.0.0:10000+T`) não colidem entre robôs.
- **Perfil `netem` (opcional).** `tc qdisc add dev eth0 root netem delay 20ms
  10ms loss 2%` em cada container de robô aproxima o Wi-Fi do ginásio. Perfil
  `wifi` no compose; padrão desligado.

## 4. Fluxo de uma partida com o GC oficial

1. `hl-referee gen-configs` → escreve `config/generated/` a partir de
   `match.yaml` (§5).
2. `docker compose up world robot1 robot3 robot5 robot2 robot4 robot6`.
3. Abrir o GameController, escolher a interface `br-hl_net`, competição
   (a divisão que o time disputa; `large_foundation` é a de 3 jogadores por time), times **56** e **57**, jogadores 1–3.
4. `hl-referee sniff` confirma `RGme v20` chegando e aceitável;
   `docker logs hl-robot1 | grep "handle v20 packet successfully"` confirma o
   nó aceitando; `tcpdump -i br-hl_net udp port 3939` mostra 6 origens a 2 Hz.
5. Operador: INITIAL → READY (robôs andam) → SET → apito (`hl-referee
   whistle`) → PLAYING no GC até 10 s depois.
6. Gol: o `world` avisa no terminal; operador registra no GC (como na
   competição). Bola fora / free kick: operador escolhe no GC e posiciona a
   bola com `hl-referee ball --free-kick <x> <y>`.
7. `hl-referee team-monitor` mostra contagem por time e tamanho máximo; tem
   que bater com o `messageBudget` que chega no pacote do GC.

## 5. `config/match.yaml` como fonte única

Campos mínimos (proposta; o gerador é `cli/gen_configs`):

```yaml
competition: large_foundation        # → players_per_team, tempos; também informa o auto_gc
game_controller:
  ip: 172.28.0.1                     # PC do árbitro; na competição, o anunciado no local
  data_port: 3838
  return_port: 3939
  accept_from: [172.28.0.1]          # → game_controller_node ip_white_list
teams:
  home: {id: 56, color: gray, attacks: +x}
  away: {id: 57, color: red,  attacks: -x}
robot_profile:                       # T1, do README do robocup_demo
  robot_height: 1.12
  odom_factor: 1.2
  enable_auto_visual_kick: false
  camera_info_topic: /boostercamera/head/rgb/camera_info
network:
  subnet: 172.28.0.0/16
  wifi_profile: none                 # none | gym (netem)
robots:
  - {name: robot1, slot: 0, team: home, player_id: 1, role: goal_keeper, ip: 172.28.0.11}
  - {name: robot3, slot: 1, team: home, player_id: 2, role: striker,     ip: 172.28.0.13}
  - {name: robot5, slot: 2, team: home, player_id: 3, role: striker,     ip: 172.28.0.15}
  - {name: robot2, slot: 3, team: away, player_id: 1, role: goal_keeper, ip: 172.28.0.12}
  - {name: robot4, slot: 4, team: away, player_id: 2, role: striker,     ip: 172.28.0.14}
  - {name: robot6, slot: 5, team: away, player_id: 3, role: striker,     ip: 172.28.0.16}
```

Gera, por robô:

`config/generated/robotN/config_local.yaml` (parâmetros do `brain_node`):

```yaml
brain_node:
  ros__parameters:
    game: {team_id: 56, player_id: 2, player_role: striker, field_type: adult_size, number_of_players: 3}
    robot: {robot_height: 1.12, odom_factor: 1.2}
    RLVisionKick: {enableAutoVisualKick: false}
    game_control_ip: "172.28.0.1"
    enable_com: true
    team_comm_frequency_hz: 1.0
```

`config/generated/robotN/game_controller.yaml` (parâmetros do
`game_controller_node`: `port`, `enable_ip_white_list`, `ip_white_list`), e o
`.env` do compose (IPs, `HL_BRAINS`). Os três números que precisam ser
iguais — `team_id` no brain, número do time no GameController e `team` no
`world` — saem do mesmo arquivo; o único que é digitado à mão é o do app.

## 6. `referee/auto_gc.py` (substituto wire-idêntico para CI e treino)

Emite `RGme` v20 de 158 B na 3838 com o mesmo header, versão, taxa (2 Hz,
5 Hz em stop), `packet_number`, `messageBudget` decrescente e o **estado
falso** de 10 s após playing, e escuta `RGrt` na 3939 e mensagens de time em
`10000+T`. Máquina de estados dirigida por script de cenário
(`tests/scenarios/*.yaml`) ou por teclas. **Não substitui o app nos ensaios
de fidelidade**; existe para o teste rodar sem operador. Toda mudança no
`protocol/gamecontroller.py` é testada contra
`game_controller_msgs/headers/RoboCupGameControlData.h` clonado do
repositório oficial.

## 7. Checklist de rede antes de dar uma partida por válida

- [ ] `gc_sniffer`: `RGme` v20, 158 B, aceitável, origem = `game_controller.ip`
- [ ] cada `hl-robotN`: log `handle v20 packet successfully` a ~2 Hz
- [ ] `tcpdump -i br-hl_net udp port 3939`: 6 origens, 32 B, 2 Hz, `teamNum`/`playerNum` corretos
- [ ] `team_monitor`: pacotes em `10056` e `10057`, ≤ 512 B, taxa ≤ 2 Hz por robô, contador igual ao do GC
- [ ] `tcpdump -i eth0 udp portrange 7400-7500` dentro de um robô: vazio (DDS não vaza)
- [ ] `ros2 node list` dentro de um robô: `brain_node`, `game_controller`, nós do emulador, nada mais
- [ ] após o apito, GC muda para PLAYING em 10 s e o brain do lado do kickoff já estava em PLAY
