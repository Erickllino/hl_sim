# Documentação do `hl_sim`

Objetivo do projeto, em uma frase: **rodar a stack de competição sem alterá-la,
substituindo apenas o que o hardware, a câmera e o campo fornecem**, para validar
estratégia sem ter os 6 robôs e o campo.

Base de código do robô a partir de agora:
[`BoosterRobotics/robocup_demo` @ `sandbox/support_2026_game_controller`](https://github.com/BoosterRobotics/robocup_demo/tree/sandbox/support_2026_game_controller)
(commit `aac541d`, "compatible with HSL game_controller v7.0").
Regras: [`HSL-Rules`](https://github.com/RoboCup-HumanoidSoccerLeague/HSL-Rules) (draft 2026).
Árbitro: [GameController oficial](https://github.com/RoboCup-HumanoidSoccerLeague/GameController) (Rust/Tauri).

## Documentos vigentes (estrutura-alvo, leia nesta ordem)

| Doc | O que responde |
|---|---|
| [`01-arquitetura-alvo.md`](01-arquitetura-alvo.md) | Princípios, fronteira de fidelidade, camadas, árvore de diretórios alvo, containers |
| [`02-contrato-emulador.md`](02-contrato-emulador.md) | Tudo que o emulador tem que publicar/consumir para o brain não perceber que não está no robô: tópicos, tipos, campos, taxas, `api_id`s, frames. Plano do modelo de marcha |
| [`03-rede-e-gamecontroller.md`](03-rede-e-gamecontroller.md) | Topologia de rede igual à da competição: portas, broadcast, orçamento de mensagens, apito, IP do árbitro, whitelist, configuração única |
| [`04-plano-refatoracao.md`](04-plano-refatoracao.md) | Ordem dos passos para sair do código atual e chegar na estrutura-alvo, com critério de aceite por passo e cenários de validação |

## Documentos históricos (contexto, não normativos)

Escritos quando a base era o fork `robocin/hsl-player`. Referências de linha
(`brain.cpp:1407`, `brain_communication.cpp:24`) e nomes de tópico
(`/booster_vision/*`) **não valem mais** para o `robocup_demo` sandbox.

- `gamecontroller-integracao.md` — decisão de o sim ser consumidor do GC (mantida)
- `27082026-knownbugs.md` — levantamento de bugs; itens 11 e 14 (portas de time) foram resolvidos pela própria Booster no sandbox
- `ROADMAP_ROS2.md`, `hl_sim_instructions.md`, `SESSAO_PROXIMA.md`, `test.md` — briefing inicial e notas de sessão
