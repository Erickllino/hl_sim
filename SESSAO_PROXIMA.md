# Problemas pendentes — próxima sessão

## Estado atual

Brain + sim estão se comunicando via ROS2 (Cyclone DDS):
- `/low_state`, `/odometer_state`, `/booster_vision/detection` publicados a 50 Hz pelo sim
- Brain processa as mensagens e publica comandos em `LocoApiTopicReq`
- Viewer do MuJoCo abre e mostra o robô

---

## Problemas não resolvidos

### 1. Robô não se move no viewer (prioridade alta)

**Sintoma:** O brain publica `kMove` com velocidades corretas mas o robô fica parado no viewer.

**Causa provável:** O sim recebe os comandos via `_on_loco` mas o tópico assinado pode não estar
sendo entregue. Verificar:
```bash
# Com sim e brain rodando:
ros2 topic echo /LocoApiTopicReq
# Confirmar que vx != 0
```

**O que já tentamos:**
- Corrigido `msg.data` → `msg.body` e `msg.header` no `_on_loco` (RpcReqMsg não tem `data`)
- `api_id` extraído do campo `header` (JSON string) e não como atributo direto

**Próximo passo:** Adicionar log em `_on_loco` para confirmar que o callback está sendo chamado:
```python
def _on_loco(self, msg):
    self._node.get_logger().info(f"loco: api_id={...} vx={...}")
```

---

### 2. Robô se move "deslizando" sem animar joints (melhoria)

**Situação:** O sim usa locomoção cinemática — só integra a posição do tronco, joints ficam fixos
em `HOME_CTRL`. O robô parece flutuar pelo campo.

**Solução planejada:** Implementar um CPG (Central Pattern Generator) simples em `sim_bridge.py`
que gere targets sinusoidais para os joints das pernas baseado na velocidade comandada.
Função `_gait_ctrl(cmd, tick)` já foi esboçada mas não implementada.

Joints relevantes:
- `[11]` LHipPitch, `[12]` LHipRoll, `[14]` LKneePitch, `[15]` LAnklePitch
- `[17]` RHipPitch, `[18]` RHipRoll, `[20]` RKneePitch, `[21]` RAnklePitch

---

### 3. Warning de VIRTUAL_ENV no sim (cosmético)

**Mensagem:** `VIRTUAL_ENV=/workspace/.venv does not match the project environment path .venv`

**Causa:** O entrypoint ativa o `.venv` mas o `uv run` espera o venv na pasta do projeto (`hl_sim/.venv`).

**Solução:** No `docker-compose.yml`, setar `UV_PROJECT_ENVIRONMENT=/workspace/.venv` para o
serviço `sim`, ou remover a ativação do venv do entrypoint para esse serviço.

---

### 4. Game controller repo ausente (baixa prioridade)

O pacote `game_controller_interface` está no workspace mas o repositório do referee (GameController)
não está configurado. O brain funciona sem ele em simulação (publica `GC_PLAYING` fixo), mas para
testes com árbitro real é necessário.

---

## Comandos úteis para a próxima sessão

```bash
# Subir tudo
xhost +local:docker
docker compose -f docker/docker-compose.yml up sim   # Terminal 1
docker compose -f docker/docker-compose.yml up brain # Terminal 2

# Verificar comunicação
ros2 node list          # deve mostrar /brain_node, /hl_sim_bridge
ros2 topic hz /low_state
ros2 topic echo /LocoApiTopicReq

# Recompilar brain (se necessário — só dentro do container dev)
docker compose -f docker/docker-compose.yml run --rm dev bash
# dentro: cd /workspace/hsl-player && source /opt/ros/humble/setup.bash
#         colcon build --symlink-install --packages-select brain
```
