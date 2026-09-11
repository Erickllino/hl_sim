# hl_sim — futebol de robôs T1

Simulador MuJoCo para desenvolver estratégias enquanto a simulação de locomoção
evolui. Os robôs deslizam com postura fixa; o chute aplica força à bola quando
há intenção de chutar e ela está ao alcance. Não há colisão robô–bola.

Cada container `robotN` executa o brain e a ponte do GameController. O container
`sim` executa o MuJoCo e publica sensores para cada robô em um domínio ROS separado.
O árbitro é o GameController oficial, executado fora destes containers.

## Rodar do zero

### 1. Pré-requisitos

- Linux com Docker Engine e Docker Compose v2 com suporte a `include` e contextos
  adicionais de build. Confira com `docker version` e `docker compose version`.
- Git e acesso ao GitHub e aos repositórios de pacotes durante o primeiro build.
- Para o viewer: sessão gráfica X11/XWayland, `DISPLAY` definido, comando `xhost`
  e dispositivo `/dev/dri`. O Compose já monta o socket X11 e esse dispositivo.
- Para partidas controladas pelo brain: [GameController oficial](https://github.com/RoboCup-HumanoidSoccerLeague/GameController),
  instalado no host ou em outro computador da rede.

ROS2, Python, MuJoCo e as dependências do brain ficam nas imagens; não é preciso
instalá-los no host para usar o fluxo Docker. Os comandos abaixo pressupõem que
seu usuário pode executar Docker.

### 2. Clonar e configurar

```bash
git clone git@github.com:Erickllino/hl_sim.git
cd hl_sim
cp .env.example .env
```

Em `.env`, deixe `HSL_PLAYER_DIR` vazio para o BuildKit clonar automaticamente
`robocin/hsl-player`, branch **main26**. Não é necessário obter o SDK interno nem
copiar o stub. Para desenvolver o brain em um clone local atualizado da main26,
coloque o caminho absoluto desse clone em `HSL_PLAYER_DIR`.

Edite `config/match.yaml` antes de gerar as configurações:

- `teams`: IDs dos times, iguais aos cadastrados no GameController.
- `robots`: jogador, função e domínio de cada robô.
- `game_controller_ip`: IP do árbitro para o retorno UDP 3939. O padrão
  `172.28.0.1` é o host visto pela bridge Docker; para outro computador, use o IP dele.

Os valores atuais são time da casa **56** e visitante **55**.

### 3. Construir as duas imagens e gerar os arquivos dos robôs

```bash
docker compose build --build-arg UID="$(id -u)" --build-arg GID="$(id -g)" robot1 sim

docker compose run --rm --no-deps --pull never sim python tools/gen_robot_configs.py
```

Os seis serviços `robotN` usam a mesma imagem `hl-robot:humble`; basta construir
um deles. A outra imagem é `hl-sim:humble`. O primeiro build baixa dependências
e pode demorar. Os arquivos gerados ficam em `config/robots/`.

Sempre que mudar `config/match.yaml`, execute o gerador novamente e reinicie os
containers de robô afetados para que o entrypoint carregue as novas configurações.

### 4. Abrir o viewer e iniciar os containers

No terminal da sua sessão gráfica:

```bash
xhost +local:docker

# Exemplo: dois brains, um de cada time.
HL_BRAINS=1,2 docker compose up -d --no-build sim robot1 robot2

# Acompanhar a execução.
docker compose ps
docker compose logs -f sim robot1 robot2
```

Para os seis brains:

```bash
HL_BRAINS=all docker compose up -d --no-build sim robot1 robot2 robot3 robot4 robot5 robot6
```

`HL_BRAINS` lista os **números dos robôs**, não os IDs dos jogadores. Use os mesmos
números dos serviços que iniciou. A cena sempre contém seis corpos. Os slots sem
brain usam agentes scriptados; eles também dependem de estado de jogo recebido
por um dos links ativos do seu time. Sem isso, permanecem parados.

Para testar apenas o MuJoCo, sem brain nem árbitro:

```bash
# Pare a simulação anterior para não abrir dois simuladores ao mesmo tempo.
docker compose stop sim

docker compose run --rm --no-deps --pull never sim \
  python -m hl_sim.sim.bridge --viewer --force-playing --duration 60
```

Esse último comando usa apenas agentes scriptados. Para rodar sem janela, retire
`--viewer`. O Compose padrão ainda requer `/dev/dri`; em um servidor sem esse
dispositivo, retire a configuração `devices` do serviço `sim`.

### 5. Conectar o GameController

No GameController oficial, configure os times com os IDs de `config/match.yaml`
e envie o broadcast UDP 3838 para a rede dos containers. A rede padrão é
`172.28.0.0/16`; seu endereço de broadcast é **172.28.255.255**.

Se o app emitir somente na rede do host, use o relay:

```bash
docker compose --profile relay up -d --no-build gc-relay
docker compose logs -f gc-relay
```

O relay encaminha os pacotes do árbitro; não cria estado de partida. Os robôs
recebem os estados `INITIAL`, `READY`, `SET` e `PLAYING` pelo GameController.
Sem os pacotes, o brain não inicia uma partida por conta própria.
O launch Docker desativa a whitelist de IPs físicos que existe no upstream.

Não inicie outro `vision_node`: o simulador fornece as detecções. Mantenha
`sim:=false` no launch do brain: este simulador ainda não publica `/clock`.

## Acessar cada container

Execute os comandos do Compose na raiz deste repositório. Use o **nome do
serviço** (`robot3`), não o nome completo do container (`hl-robot3`).

| Serviço | Container | ROS_DOMAIN_ID | Time atual | Jogador | Função |
|---|---|---:|---:|---:|---|
| `robot1` | `hl-robot1` | 1 | 56 | 1 | goleiro |
| `robot3` | `hl-robot3` | 3 | 56 | 2 | atacante |
| `robot5` | `hl-robot5` | 5 | 56 | 3 | atacante |
| `robot2` | `hl-robot2` | 2 | 55 | 1 | goleiro |
| `robot4` | `hl-robot4` | 4 | 55 | 2 | atacante |
| `robot6` | `hl-robot6` | 6 | 55 | 3 | atacante |
| `sim` | `hl-sim` | vários contextos | ambos | — | MuJoCo |

Por exemplo, para entrar no robô 3:

```bash
docker compose exec robot3 bash

# Dentro do container: docker exec não executa novamente o entrypoint.
source /opt/ros/humble/setup.bash
source "$HSL_DIR/install/setup.bash"
echo "$ROS_DOMAIN_ID"  # deve mostrar 3
```

Troque `robot3` por qualquer outro serviço ativo. `exit` sai do terminal sem
parar o container.

Para entrar no simulador e consultar o domínio do robô 3:

```bash
docker compose exec -e ROS_DOMAIN_ID=3 sim bash
source /opt/ros/humble/setup.bash
source "$HSL_DIR/install/setup.bash"
source /opt/venv/bin/activate
```

A variável do terminal não altera os domínios do processo MuJoCo que já está
rodando; ela seleciona o robô visto pelos comandos ROS desse terminal.

Também existe um container de diagnóstico que já carrega o ambiente ROS:

```bash
DOMAIN=3 docker compose --profile debug run --rm --pull never doctor
```

## Inspecionar nós, tópicos e mensagens

Dentro de um dos terminais ROS acima:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /LocoApiTopicReq -v
ros2 topic echo /LocoApiTopicReq
ros2 topic echo /kick_intent
ros2 topic echo /robocup/game_controller --once
ros2 topic hz /low_state
ros2 topic hz /booster_soccer/detection
ros2 interface show booster_msgs/msg/RpcReqMsg
```

`Ctrl+C` encerra a inspeção atual. Para rodar um comando diretamente do host:

```bash
docker compose exec robot3 bash -c \
  'source /opt/ros/humble/setup.bash; source "$HSL_DIR/install/setup.bash"; ros2 topic list -t'
```

| Tópico | Direção | Tipo / uso |
|---|---|---|
| `/LocoApiTopicReq` | brain → sim | `booster_msgs/msg/RpcReqMsg`: movimento e cabeça |
| `/kick_intent` | brain → sim | `std_msgs/msg/Bool`: Kick normal ativo |
| `/low_state` | sim → brain | `booster_interface/msg/LowState` |
| `/odometer_state` | sim → brain | `booster_interface/msg/Odometer` |
| `/head_pose` | sim → brain | `geometry_msgs/msg/Pose` |
| `/booster_soccer/detection` | sim → brain | `vision_interface/msg/Detections` |
| `/booster_soccer/line_segments` | sim → brain | `vision_interface/msg/LineSegments` |
| `/robocup/game_controller` | ponte GC → brain/sim | estado do árbitro |

Os nomes são iguais para todos os robôs. Quem separa os grafos é o
`ROS_DOMAIN_ID`, não um prefixo como `/robot3/`.

## Enviar comandos manualmente a um robô

Use o terminal do robô escolhido. Para o brain não sobrescrever seus comandos,
pause **somente o processo brain** enquanto faz o teste:

```bash
pkill -STOP -x brain_node

# Parar o movimento que já estava ativo.
ros2 topic pub --once /LocoApiTopicReq booster_msgs/msg/RpcReqMsg \
  '{uuid: "manual", header: "{\"api_id\":2001}", body: "{\"vx\":0.0,\"vy\":0.0,\"vyaw\":0.0}"}'

# Avançar a 0,2 m/s. O comando permanece ativo até receber outra velocidade.
ros2 topic pub --once /LocoApiTopicReq booster_msgs/msg/RpcReqMsg \
  '{uuid: "manual", header: "{\"api_id\":2001}", body: "{\"vx\":0.2,\"vy\":0.0,\"vyaw\":0.0}"}'

# Cabeça: ângulos em radianos.
ros2 topic pub --once /LocoApiTopicReq booster_msgs/msg/RpcReqMsg \
  '{uuid: "manual", header: "{\"api_id\":2004}", body: "{\"yaw\":0.0,\"pitch\":0.3}"}'

# Manter intenção de chute enquanto aproxima o robô da bola.
ros2 topic pub -r 20 /kick_intent std_msgs/msg/Bool '{data: true}'
```

A intenção gera **um** impulso quando a bola está no alcance. Pressione `Ctrl+C`
e publique `false` antes de iniciar outra intenção. Sem atualização por 250 ms,
a intenção também expira. Para concluir o teste:

```bash
ros2 topic pub --once /kick_intent std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /LocoApiTopicReq booster_msgs/msg/RpcReqMsg \
  '{uuid: "manual", header: "{\"api_id\":2001}", body: "{\"vx\":0.0,\"vy\":0.0,\"vyaw\":0.0}"}'

# Retomar a estratégia.
pkill -CONT -x brain_node
```

Não use `docker pause` neste teste: ele congela também os terminais do container.
Esses comandos pressupõem o `robot_name` vazio da configuração padrão main26.

## Ajustar o chute e desenvolver

Edite `config/simulation.yaml`: o padrão é **10 N por 0,1 s**, alcance de
**0,45 m**, bola à frente (±60°) e até 0,3 m de altura. Reinicie `sim` depois de
alterar os valores. O impulso é horizontal e segue a orientação do robô no início
do chute. Colisões com chão e gols continuam ativas.

O build aplica `docker/patches/main26-kick-intent.patch` ao brain para publicar a
intenção do nó `Kick`, que originalmente só envia velocidade. `/kick_ball` não é
o gatilho: esse tópico contém referências mesmo quando não há chute em execução.
Detalhes: [main26 e chute simplificado](docs/main26-simulacao.md).

- Alterou Python em `src/hl_sim/` ou a configuração do chute: `docker compose restart sim`.
- Alterou o brain local: configure `HSL_PLAYER_DIR` e reconstrua `robot1`.
- Alterou mensagens ROS ou atualizou a branch: reconstrua **robot1 e sim**.
- Após reconstruir, use `docker compose up -d --no-build ...` com os serviços
  ativos para recriar seus containers. Um simples `restart` não troca a imagem.

Testes de física e comunicação ROS:

```bash
docker compose run --rm --no-deps --pull never sim \
  python -m unittest discover -s /workspace/tests -v
```

Teste do brain compilado e do sinal de intenção:

```bash
docker run --rm --network none \
  --mount type=bind,src="$PWD",dst=/workspace,readonly \
  --entrypoint bash hl-robot:humble -c \
  'source /opt/ros/humble/setup.bash; source "$HSL_DIR/install/setup.bash"; python3 /workspace/tests/check_brain_kick_intent.py'
```

## Parar e diagnosticar

```bash
docker compose logs --tail 100 robot3
docker compose logs --tail 100 sim
docker compose restart robot3
docker compose stop robot3
docker compose down
```

- **Robô não vê a bola:** confira `/booster_soccer/detection` no domínio correto.
- **Robô parado:** confira os IDs de time, `/robocup/game_controller` e o estado
  do árbitro; verifique também se o brain foi pausado em um teste manual.
- **Tópicos vazios:** confira `ROS_DOMAIN_ID`, source dos ambientes e se os serviços
  estão ativos na rede `hl_net`.
- **Viewer falha:** confira `DISPLAY`, `xhost` e `/dev/dri`. Em máquinas NVIDIA,
  pode ser necessário configurar o runtime de GPU indicado em `docker/compose.yaml`.
- **Patch não aplica:** o upstream mudou; atualize o patch para a revisão usada
  ou selecione a revisão validada indicada no documento da migração.

Ao terminar de usar o viewer, pode revogar a permissão com `xhost -local:docker`.
