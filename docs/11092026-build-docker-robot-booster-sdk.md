# Build do container `robot` (Docker) — 11/09/2026

Investigação de por que `docker compose -f docker/compose.yaml up` não builda o
container `robot` (brain + game_controller do `hsl-player`). Quatro problemas
foram encontrados em cadeia — resolver um só revela o próximo. Três já têm fix
aplicado; o quarto está **bloqueado** e precisa de uma decisão de arquitetura.

---

## 1. `HSL_REPO` apontava pro repositório errado  ✅ RESOLVIDO

**Arquivo:** `docker/Dockerfile:74`

O Dockerfile clonava `git@github.com:BoosterRobotics/robocup_demo.git` na
branch `competition-alan`. Essa branch não existe mais nesse repo (`git
ls-remote --heads` não lista nada parecido, nem em tags) — o hsl-player
migrou para `github.com/robocin/hsl-player`, onde `competition-alan` existe
(confirmado via `git ls-remote`).

**Fix:** `HSL_REPO=git@github.com:robocin/hsl-player.git`. `HSL_REF` continua
`competition-alan` (valor já correto, só o repo estava errado).

Confirmado também que o include do brain relevante pro item 4 abaixo é
**idêntico** entre `robocin/hsl-player@competition-alan` e
`BoosterRobotics/robocup_demo@sandbox/support_2026_game_controller` — quando o
projeto trocar de branch/repo (planejado, ver conversa), esse problema não
muda de forma.

---

## 2. `libarrow-dev` não existe nos repos padrão do Ubuntu  ✅ RESOLVIDO

**Arquivo:** `docker/Dockerfile:110-120` (novo `RUN` antes do install de deps
do brain)

`apt-get install ... libarrow-dev` falha com `E: Unable to locate package`.
Confirmado rodando `ubuntu:jammy` limpo: `apt-cache policy libarrow-dev` não
retorna nada em `main`/`universe`/`multiverse` nem no mirror do ROS. A Apache
Arrow só distribui via apt-repo próprio.

**Fix:** baixar e instalar
`apache-arrow-apt-source-latest-<codename>.deb` (jfrog, oficial da Apache
Arrow) antes do `apt-get install` que pede `libarrow-dev`.

---

## 3. Bug de empacotamento do `ros-humble-behaviortree-cpp` no Jammy  ✅ RESOLVIDO

**Arquivo:** `docker/Dockerfile:132-139` (novo `RUN` de symlink)

Erro: `CMake Error ... Package 'behaviortree_cpp' exports the library
'behaviortree_cpp' which couldn't be found`.

Causa raiz (lida direto no `.cmake` instalado pelo pacote apt): o
`.so` vai para `/opt/ros/humble/lib/x86_64-linux-gnu/` (path multiarch), mas
`ament_cmake_export_libraries-extras.cmake` procura com `find_library(...
NO_DEFAULT_PATH)` restrito a `/opt/ros/humble/lib` direto — nunca olha o
subdiretório multiarch. É bug de empacotamento do pacote apt (`4.9.1-1jammy`),
não do `hsl-player`.

**Fix:** depois de instalar o pacote, symlink de tudo em
`lib/x86_64-linux-gnu/*.so*` para dentro de `lib/` direto.

---

## 4. Falta o SDK interno da Booster Robotics  🔴 BLOQUEADO — precisa decisão

**Erro:** `brain.h:16:10: fatal error: booster/robot/b1/b1_api_const.hpp: No
such file or directory`, e (depois de resolver isso) símbolos como
`booster_internal::robot::b1::LocoInternalApiId::kRLKickBall` não resolvem.

### O que é

O brain **não calcula a marcha**. Ele manda comandos de alto nível
(`MoveToTarget`, `SquatAction`, `EnableRobocupWalkMode`, `RLKickBall`,
`GoalieSquatDown`, `HighKick`, `ChangeControlMode`, etc.) via RPC pro
controlador de locomoção que roda dentro do robô real (firmware/placa de
controle — a policy de marcha embarcada). Essa API vem de duas árvores de
headers + uma lib estática, todas fora do repo do `hsl-player`:

- `booster::robot::b1` (público) — `b1_api_const.hpp` e cia.
- `booster_internal::robot::b1` (**interno**) — `LocoInternalApiId`,
  `B1LocoInternalClient`, usado pelo `robot_client.cpp` do brain pra quase
  tudo que interessa pro jogo (chute, squat, modo robocup walk...).

### O que já foi checado

- **SDK público no GitHub** (`github.com/BoosterRobotics/booster_robotics_sdk`,
  branch `main`, aberto): tem `booster::robot::b1` mas **não tem**
  `booster_internal` — nenhum símbolo (`kRLKickBall`, `kSquatAction`,
  `kEnableRobocupWalkMode`, `kGoalieSquatDown`, `kEnableVisualKickMode`,
  `kRLFancyKickBall`, `kMoveToTargetInternal`, ...) existe lá. **Não dá pra
  usar sozinho.**

- **Robô real** (`booster@192.168.0.102`, acessado via SSH): tem tudo.
  ```
  /usr/local/include/booster/                              (público)
  /usr/local/include/booster_internal/robot/b1/b1_loco_internal_client.hpp
  /usr/local/include/booster_internal/robot/b1/b1_loco_internal_api.hpp
  /usr/local/lib/libbooster_robotics_sdk.a
  ```

- **Arquitetura:** `uname -m` no robô → `aarch64` (Jetson). O build do
  container `robot` roda em host x86_64. Uma `.a` aarch64 não linka em
  x86_64.

- **`b1_loco_internal_client.hpp` não é header-only.** A maioria dos métodos
  (`SquatAction`, `RLKickBall`, `MoveToTarget`...) são inline e só chamam
  `SendApiRequest(ApiId, json)` — mas `SendApiRequest(...)` e os dois
  `Init(...)` **não têm corpo no header**. `ar t libbooster_robotics_sdk.a |
  grep internal` confirma: existe um objeto compilado
  `b1_loco_internal_client.cpp.o` dentro da lib. Não temos o `.cpp` fonte,
  só o `.o` aarch64 — não dá pra recompilar pra x86_64 sem o código-fonte
  real da Booster.

- **Histórico do próprio repo:** o usuário lembrava de ter feito um stub
  disso numa sessão anterior. Chequei todo o histórico (`git log --all`,
  branch `HLRoboSim`, e o `stash@{0}` em cima dela) — não achei nenhum
  arquivo relacionado a `booster_internal`/stub. O diff de `HLRoboSim` é só
  reorganização de diretórios (`assets/` → `src/assets/` etc.); o stash só
  tem 1 linha (a mesma correção do item 1). Ou não foi commitado, ou foi em
  outra máquina/pasta.

### Caminhos possíveis (não decidido ainda)

1. **Stub próprio compilado localmente** — escrever
   `b1_loco_internal_client.cpp` (e o que mais for preciso do lado público)
   implementando só `Init()`/`SendApiRequest()` como no-op (loga e retorna
   0), usando os headers reais copiados do robô (que definem as classes,
   enums e structs de parâmetro — isso não é código proprietário compilado,
   só declaração). Compila nativo em x86_64, sem depender da `.a` da Jetson.
   Destrava o build agora; a ligação de verdade com o MuJoCo (traduzir
   `RLKickBall`/`MoveToTarget`/etc. em ação simulada) fica pra depois — é
   exactly o ponto de integração que o `hl_sim` precisa imitar (ver
   `docs/hl_sim_instructions.md`, Fase 2 e 5).
2. **Buildar o container `robot` pra `linux/arm64`** via `docker buildx` +
   QEMU, usando a `.a` real vendorizada do robô. Byte-compatível com o
   firmware real, mas brain roda emulado (mais lento) num host x86_64. Não
   cheguei a testar se `buildx`/QEMU já estão instalados nessa máquina.
3. Perguntar pra equipe/Booster se existe build x86_64 do SDK interno —
   não verificado, sem garantia de existir.

**Nada disso foi implementado ainda** — o usuário pediu pra documentar antes
de decidir.

---

## Estado atual do `docker/Dockerfile`

- Itens 1 (repo/branch) — no `HEAD` do branch `test`.
- Itens 2 e 3 (Apache Arrow + symlink behaviortree_cpp) — **aplicados no
  working tree, ainda não commitados** (`git diff docker/Dockerfile` mostra
  as duas seções novas, +21 linhas).
- Item 4 — nada aplicado, aguardando decisão acima.

Com 1–3 aplicados, o build chega até compilar `game_controller` com sucesso
e só falha no `brain` por causa do item 4.
