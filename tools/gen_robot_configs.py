#!/usr/bin/env python3
"""
tools/gen_robot_configs.py — gera o config_local.yaml de cada container de robô.

O brain lê `config.yaml` e depois `config_local.yaml`, que sobrescreve o primeiro
(brain/launch/launch.py:17).  Esse segundo arquivo não existe no repo do
hsl-player: é o gancho de override por máquina.  É onde entram team_id e
player_id — os números que precisam bater com o cadastro no GameController, sob
pena de brain.cpp:1407 descartar o pacote inteiro e o robô ficar em INITIAL para
sempre, sem erro visível.

Fonte única: config/match.yaml.  Saída: config/robots/<nome>.yaml, montado em
/etc/hl/config_local.yaml pelo compose.

    python tools/gen_robot_configs.py
    python tools/gen_robot_configs.py --check    # só confere se está atualizado
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from hl_sim import config as match_config  # noqa: E402
from hl_sim import paths                   # noqa: E402

HEADER = """# GERADO por tools/gen_robot_configs.py — não edite à mão.
# Fonte: config/match.yaml.  Sobrescreve campos do config.yaml do brain.
#
# container : {name}
# domínio   : ROS_DOMAIN_ID={domain}
# slot      : {slot}  (índice em scenes/soccer_scene.xml)
"""


def render(robot) -> str:
    return HEADER.format(name=robot.name, domain=robot.domain_id, slot=robot.slot) + f"""
brain_node:
  ros__parameters:
    game:
      team_id: {robot.team_id}
      player_id: {robot.player_id}
      player_role: "{robot.role}"
"""


def main() -> None:
    ap = argparse.ArgumentParser(description="gera os config_local.yaml dos robôs")
    ap.add_argument("--check", action="store_true",
                    help="não escreve; sai com código 1 se algo estiver desatualizado")
    args = ap.parse_args()

    cfg = match_config.load()
    out_dir = paths.CONFIG / "robots"
    out_dir.mkdir(parents=True, exist_ok=True)

    stale = []
    for robot in cfg.robots:
        target = out_dir / f"{robot.name}.yaml"
        content = render(robot)
        current = target.read_text(encoding="utf-8") if target.exists() else None

        if current == content:
            print(f"  = {target.relative_to(paths.ROOT)}")
            continue

        stale.append(target)
        if args.check:
            print(f"  ! {target.relative_to(paths.ROOT)} desatualizado")
            continue

        target.write_text(content, encoding="utf-8")
        print(f"  → {target.relative_to(paths.ROOT)}  "
              f"team {robot.team_id} · player {robot.player_id} · {robot.role}")

    if args.check and stale:
        print(f"\n{len(stale)} arquivo(s) desatualizado(s). "
              f"Rode: python tools/gen_robot_configs.py", file=sys.stderr)
        raise SystemExit(1)

    if not args.check:
        print(f"\nPronto. Os nomes de time precisam bater com o cadastro no "
              f"GameController: {cfg.home_team_id} e {cfg.away_team_id}.")


if __name__ == "__main__":
    main()
