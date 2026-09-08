# 04 — Plano de refatoração

> Do código de hoje (branch `test`) até a arquitetura de
> `01-arquitetura-alvo.md`. Cada passo termina com um critério de aceite
> verificável e deixa o repositório utilizável. A ordem prioriza o que o
> usuário pediu: **o emulador funcionando com a base nova o quanto antes**;
> realismo de marcha vem depois.

## Passo 0 — Congelar o estado atual

- Tag `v0-hsl-player` no commit atual.
- `docs/README.md` marca os docs antigos como históricos (feito).

Aceite: `git tag` mostra a tag; nada mais muda.

## Passo 1 — Trocar a base do container do robô

O que muda: `docker/Dockerfile` (target `robot`) e `docker/launch/robot.launch.py`.

- Clonar por HTTPS `https://github.com/BoosterRobotics/robocup_demo.git`,
  branch `sandbox/support_2026_game_controller`, pinado por commit
  (`aac541d…`). Sem `--ssh`, sem chave.
- Clonar `https://github.com/BoosterRobotics/booster_robotics_sdk.git`
  (headers em `include/booster/robot/...` e `lib/x86_64/libbooster_robotics_sdk.a`);
  o brain inclui `<booster/robot/b1/b1_loco_api.hpp>`.
- `colcon build --packages-select booster_msgs booster_interface
  vision_interface game_controller_interface game_controller brain`
  (a pasta é `src/interface/booster_ros2_interface`, o pacote é
  `booster_interface`). Não buildar `vision` nem `whistle_detection`.
- Dependências apt que o brain pede: `ros-humble-behaviortree-cpp`,
  `ros-humble-backward-ros`, tf2, Eigen, OpenCV, yaml-cpp, Rerun SDK
  (verificar versão no `CMakeLists.txt` do brain).
- Workspace em `/home/booster/Workspace/robocup_demo`, usuário `booster`.
- `FASTRTPS_DEFAULT_PROFILES_FILE` → `configs/fastdds.xml` **do repo
  clonado**. Remover `docker/fastdds-container.xml`. Enquanto o emulador ainda
  rodar no `world` (até o Passo 4), o container precisa de uma exceção
  temporária: perfil sem whitelist. Marcar com `TODO(passo-4)`.
- `robot.launch.py` = `game_controller/launch/launch.py` +
  `brain/launch/launch.py` com `sim:=false`, `disable_com:=false`, sem
  `vision`, sem `whistle`.

Aceite:
- `docker compose build robot1` termina sem patch no `robocup_demo`.
- Dentro do container: `ros2 node list` → `/brain_node`, `/game_controller`.
- `ros2 topic info /LocoApiTopicReq` mostra 1 publisher (brain).
- `docker logs hl-robot1` mostra `GameControl IP:` e `Team broadcast initialized on port 10056`.

## Passo 2 — `match.yaml` v2 e gerador único

O que muda: `config/match.yaml`, `src/hl_sim/config.py`,
`tools/gen_robot_configs.py` → `cli/gen_configs.py`, `docker/compose.yaml`.

- Esquema de `03-rede-e-gamecontroller.md` §5: `game_controller.ip`,
  `accept_from`, `robot_profile`, IP por robô, `competition`.
- Gerar `config_local.yaml` (brain), `game_controller.yaml` (nó do GC) e
  `.env` (compose). `config/generated/` no `.gitignore`.
- O `robot.launch.py` passa `game_controller.yaml` como `parameters=` do nó
  do GC (o launch da Booster hardcoda a whitelist; o nosso launch **inclui o
  nó diretamente** com os mesmos `executable`/`name`, só para injetar o
  arquivo de parâmetros; isso não altera o código da Booster).

Aceite:
- `gen_configs --check` limpo.
- Com o GC oficial no host em `br-hl_net`: `docker logs hl-robot1 | grep "handle v20 packet successfully"` a ~2 Hz.
- `sudo tcpdump -i br-hl_net -c 12 udp port 3939` mostra 6 IPs distintos.

## Passo 3 — Alinhar o `ROS2Agent` atual com o contrato novo (ponte rápida)

O objetivo deste passo é **voltar a ter o brain andando** com a base nova
antes de mover código de lugar. Só edita `src/hl_sim/ros2/agent.py`,
`game_controller_link.py`, `protocol/gamecontroller.py`.

- Tópicos: `/booster_vision/detection` → `/booster_soccer/detection`;
  `/booster_vision/line_segments` → `/booster_soccer/line_segments`.
- Publicar `camera_info` (rgb e depth, latched) com a intrínseca do T1;
  publicar `fall_down_recovery_state` = READY a 1 Hz.
- `api_id`s: manter 2001/2004/2008; **adicionar** 2000 (`kChangeMode`) e
  2038 (`kVisualKick {start}`); **remover** 2024 e os `10000x` do fork.
- Linhas de campo: publicar os segmentos do campo visíveis no FOV (versão
  simples: todas as linhas dentro de 8 m e ±45° da cabeça, em frame do robô e
  em pixel). Sem isso o `locator` do brain não localiza.
- Odometria: publicar `verdade / odom_factor` (1.2). Deriva fica para o
  Passo 6.
- `protocol/gamecontroller.py`: tabela v19 para o que chega no tópico;
  `GameState.penalties` em v19; manter v20 para o fio (sniffer, auto_gc).
- `/whistle_detected`: publicar a partir de um evento do `run_ros2`
  (tecla `w` no terminal) para todos os domínios. Versão provisória do
  `referee/whistle`.

Aceite (cenário `kickoff`, manual, com o GC oficial):
- INITIAL: robôs parados. READY: os com brain andam para a posição; SET: param.
- Tecla `w` em SET: brain do lado do kickoff loga `Whistle detected in SET state`;
  operador põe PLAYING no GC até 10 s depois; robô vai à bola.
- `ros2 topic hz /booster_soccer/detection` ≈ 30 Hz no domínio do robô.

## Passo 4 — Separar `world` / `link` / `emulator` e confinar o DDS

O passo estrutural. Move código, não muda comportamento observável do brain.

- `sim/bridge.py` + `sim/layout.py` → `world/` (`physics.py`, `model.py`,
  `server.py`). `agents/` fica. Sai todo `import rclpy` de fora de `emulator/`.
- `link/schema.py` (dataclasses `RobotState`, `WorldState`, `RobotCmd`,
  `RefereeCmd`, `SCHEMA_VERSION`) e `link/transport.py` (ZMQ PUB/SUB para
  estado a 50 Hz, REQ/REP para comandos e ações do árbitro).
- `ros2/agent.py` + `ros2/robot_link.py` + `ros2/game_controller_link.py` →
  `emulator/firmware_node.py`, `emulator/vision_node.py`,
  `emulator/whistle_node.py`, `emulator/launch/emulator.launch.py`. O
  `GameControllerLink` deixa de existir no emulador: quem precisa do estado de
  jogo fora do brain são os agentes scriptados, que passam a ouvir a 3838
  diretamente no `world` via `protocol/` (o `world` está na mesma bridge e
  recebe o broadcast).
- Target Docker `emulator` (Python + rclpy + as 4 interfaces) instalado na
  imagem `hl-robot`; `robot.launch.py` inclui `emulator.launch.py`.
- Remover `ROS_DOMAIN_ID` do compose. Remover o perfil FastDDS temporário do
  Passo 1: passa a valer o `configs/fastdds.xml` da Booster.
- `world` vira serviço próprio (`hl-world`, 172.28.0.10), sem ROS2 na imagem.

Aceite:
- `grep -r "import rclpy" src/hl_sim | grep -v emulator` vazio (vira teste).
- Dentro de `hl-robot1`: `tcpdump -i eth0 -c 5 udp portrange 7400-7500` não captura nada em 10 s.
- `ros2 node list` no robô: `/brain_node`, `/game_controller`, `/hl_firmware`, `/hl_vision`, `/hl_whistle`.
- Cenário `kickoff` do Passo 3 continua passando.

## Passo 5 — Lado do árbitro

- `referee/whistle.py`, `referee/ball.py`, `referee/pickup.py` falando
  `RefereeCmd` com o `world`; `referee/team_monitor.py` escutando
  `10000+T` em `br-hl_net`; `referee/auto_gc.py` (RGme v20 wire-idêntico,
  estado falso de 10 s, orçamento); `gc_sniffer`/`gc_relay` movidos para cá.
- Imagem `hl-referee` com `network_mode: host`; CLI única `hl-referee
  <subcomando>`.
- `world/events.py`: gol, bola fora, robô fora, robô caído → mensagem no
  terminal do árbitro (nunca muda estado de jogo).

Aceite:
- `hl-referee team-monitor` conta o mesmo que o GC oficial mostra na UI para 2 min de jogo.
- `tests/scenarios/kickoff.yaml` roda em CI com `auto_gc`, sem operador, e termina com a bola tocada pelo lado do kickoff antes dos 10 s de "ball free".

## Passo 6 — Modelos de erro

`emulator/models/`: odometria (fator + deriva), câmera (FOV real, 30 Hz,
latência, ruído por distância, falso negativo, oclusão), apito (atraso e
perda), queda (`world/events.py` + `kGetUp` com duração). Flag
`--perfect` desliga tudo.

Aceite: com `--perfect` o cenário `kickoff` é idêntico ao Passo 5; sem a
flag, o `locator` do brain converge (log `locate success`) em até 45 s de
READY.

## Passo 7 — Locomoção L1 e L2

- `locomotion/base.py` como interface; `kinematic.py` (L0) é o padrão.
- `tools/record_real_robot.sh` (rosbag2 com a lista de
  `02-contrato-emulador.md` §6) e `tools/fit_response_model.py` → L1.
- L2 (`learned.py`, RNN) entra quando houver gravação suficiente; o
  contrato não muda.

Aceite L1: erro de tempo de chegada a um ponto a 3 m, comparado ao robô
real, < 15 %.

## Passo 8 — Testes e CI

- `tests/test_protocol_gc.py`: `struct` de `protocol/` vs
  `RoboCupGameControlData.h` oficial (baixado no CI); round-trip
  `encode(decode(x)) == x`; tamanho 158/32 B.
- `tests/test_msg_contract.py`: campos e tipos das mensagens que o emulador
  publica vs `.msg` do `robocup_demo` clonado (falha se a Booster mudar a
  interface).
- `tests/test_no_rclpy_outside_emulator.py`.
- `tests/scenarios/`: `kickoff`, `opponent_kickoff`, `brief_stop`,
  `standard_penalty` (retirada, 45 s, reentrada), `free_kick`
  (posicionamento da bola, avoidance), `message_budget` (forçar
  `team_comm_frequency_hz: 2` e verificar contagem), `gc_loss` (auto_gc para
  de emitir por 20 s), `half_time` (troca de lado). Todos headless com
  `auto_gc` e agentes scriptados como adversários; os com brain exigem a
  imagem `hl-robot` no CI.

## Ordem e dependências

```
0 ─ 1 ─ 2 ─ 3 ─┬─ 4 ─ 5 ─ 8
               └─ 6 ─ 7
```

3 é o primeiro ponto em que a base nova joga. 4 pode começar em paralelo a 6.
Nada em 6 e 7 depende de 5.

## Checklist de fidelidade (o que "fiel à competição" significa aqui)

- [ ] Nenhum arquivo do `robocup_demo` é modificado; `config_local.yaml` é o único override e contém só o que a Booster documenta como configurável.
- [ ] DDS não sai do container do robô.
- [ ] Os únicos fluxos que saem do robô são UDP 3838 (entra), 3939 (sai), `10000+T` (broadcast) e o `link` do emulador (que representa o barramento interno do robô).
- [ ] Estado de jogo só vem de um pacote `RGme` v20 parseado pelo `game_controller_node` real.
- [ ] Apito, gol, bola fora, penalidade e reentrada exigem uma ação humana (ou do script de cenário), nunca são automáticos no modo `match`.
- [ ] Tempo de parede, 1×, sem `/clock`.
- [ ] Odometria, visão e apito têm erro modelado e ajustado em dados do robô real; `--perfect` existe só para depurar.
- [ ] `team_monitor` e o GC oficial concordam na contagem de mensagens.
