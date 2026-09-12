# main26 e chute simplificado

O Compose usa o contexto Git `git@github.com:robocin/hsl-player.git#main`.
Em 12/09/2026, a antiga branch `main26` passou a estar em `main`, com o mesmo
commit. O contexto usa a autenticação do agente SSH encaminhado pelo Compose.
A versão validada nesta migração é `68e1b79c9166c8346f2ee39c62acc33e9931e80a`.
`HSL_PLAYER_DIR` vazio seleciona esse contexto; um caminho local substitui a
fonte, e deve apontar para um checkout atualizado de main. Para reproduzir
exatamente uma revisão, também é possível usar nesse campo uma URL Git com
`#<commit completo>`. A branch remota pode avançar entre builds.

O brain desta branch usa apenas o SDK público da Booster. O stub e sua lógica
de instalação foram removidos; o Dockerfile não depende de `vendor/`.
A visão agora usa `/booster_soccer/detection` e
`/booster_soccer/line_segments`. Reconstrua **robot e sim** para usar as mesmas
interfaces ROS:

```sh
docker compose build robot1 sim
docker compose up -d --no-build sim robot1
```

Adicione os outros serviços robot desejados ao `up`. Os robôs continuam
aguardando o GameController conforme o fluxo normal da partida.

O launch Docker recebe UDP 3838 sem a whitelist de IPs físicos do upstream.
O endereço de retorno do brain ao árbitro é configurado por
`game_controller_ip` em `config/match.yaml` e propagado pelo gerador.

## Intenção de chute

O `Kick` normal da main26 só publica velocidade. `/kick_ball` contém referências
publicadas sempre que há bola detectada, inclusive fora de um chute; por isso
não serve como gatilho.

`docker/patches/main26-kick-intent.patch` adiciona ao brain, **durante o build**,
a publicação `std_msgs/Bool` em `/kick_intent`: verdadeiro enquanto o nó `Kick`
está executando; falso na saída, interrupção ou cancelamento. A marcha e as
decisões do brain permanecem as da main26. O checkout original não é alterado.
O patch também cobre o `Kick` usado para passes (`cross`) e chute assistido.
Uma mudança incompatível no upstream interrompe a aplicação do patch no build.

O `ROS2Agent` transforma essa intenção em `ActionCmd.shoot`. Mensagens repetidas
mantêm a intenção ativa; cada nova ativação recebe um identificador. Se o brain
parar de publicar por 250 ms, a intenção expira. O simulador não deduz chute de
velocidade, proximidade ou mera detecção da bola.

## Força e alcance

Edite `config/simulation.yaml` e reinicie o serviço `sim`:

- `force_newtons`: 10 N inicialmente.
- `duration_seconds`: 0,1 s (arredondada para cima em ticks de 20 ms).
- `distance_m`: 0,45 m no plano, medidos do centro do tronco à bola.
- `max_angle_degrees`: bola até 60° para cada lado da frente do robô.
- `max_ball_height_m`: bola até 0,3 m acima do chão.

Com intenção ativa e bola alcançável, aplica-se uma força horizontal na direção
em que o robô está virado, fixada no início do impulso. Isso equivale inicialmente
a um impulso de 1 N·s; não define diretamente a velocidade final da bola.
Um chute já iniciado termina seu pulso mesmo se a intenção for cancelada.
A mesma intenção não repete o impulso. Uma intenção fora do alcance pode esperar
o robô chegar perto, desde que continue ativa. Não há fila após cancelamento.

Os robôs continuam deslizando com postura fixa. Colisões **robô–bola** são
desativadas no modelo carregado pelo SimBridge; bola–chão e bola–gols são
preservadas. O chute depende da intenção e do alcance, não de contato físico.

## Verificação

```sh
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -v
```

Os testes de MuJoCo cobrem alcance, direção, altura, ausência de intenção,
impulso único, reativação, configuração e movimento real da bola sem contato.
Os testes ROS, quando executados na imagem sim com seu ambiente carregado,
cobrem entrega da intenção, expiração e os tópicos de visão.
