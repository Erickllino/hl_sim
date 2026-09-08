"""
hl_sim/config.py — carrega config/match.yaml.

Existe para matar a falha silenciosa nº1 do projeto: `team_id` e `player_id`
viviam como default de argparse em run_ros2.py e tinham que bater, de cabeça,
com o config.yaml do brain e com o cadastro no GameController.  Agora saem todos
de um arquivo só, que também gera os config_local.yaml dos containers.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml

from hl_sim import paths


@dataclass(frozen=True)
class RobotConfig:
    name:      str    # nome do corpo na cena e do container ("robot3")
    slot:      int    # índice na ordem dos <include> da cena — indexa qpos/ctrl
    team:      str    # "home" | "away"
    team_id:   int    # número cadastrado no GameController
    player_id: int    # 1-based, como no protocolo e no config.yaml do brain
    role:      str    # "striker" | "goal_keeper"
    domain_id: int    # ROS_DOMAIN_ID do container deste robô

    @property
    def number(self) -> int:
        """Número do robô ('robot3' → 3).  É também o domain_id, por convenção."""
        return int(self.name.removeprefix("robot"))


@dataclass(frozen=True)
class MatchConfig:
    home_team_id: int
    away_team_id: int
    robots: tuple[RobotConfig, ...]

    def by_name(self, name: str) -> RobotConfig:
        for r in self.robots:
            if r.name == name:
                return r
        raise KeyError(f"robô '{name}' não está em {paths.MATCH_CONFIG}")

    def by_number(self, number: int) -> RobotConfig:
        return self.by_name(f"robot{number}")

    def by_slot(self, slot: int) -> RobotConfig:
        for r in self.robots:
            if r.slot == slot:
                return r
        raise KeyError(f"slot {slot} não está em {paths.MATCH_CONFIG}")

    def opponent_team_id(self, team_id: int) -> int:
        return self.away_team_id if team_id == self.home_team_id else self.home_team_id


def load(path: Optional[Path] = None) -> MatchConfig:
    path = paths.require(path or paths.MATCH_CONFIG, "config da partida")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))

    teams = raw["teams"]
    team_ids = {"home": int(teams["home"]), "away": int(teams["away"])}

    robots = []
    seen_slots: set[int] = set()
    seen_domains: set[int] = set()
    for entry in raw["robots"]:
        team = entry["team"]
        if team not in team_ids:
            raise ValueError(f"{path}: time '{team}' desconhecido em {entry['name']}")

        slot, domain = int(entry["slot"]), int(entry["domain_id"])
        # Slot duplicado = dois agentes escrevendo no mesmo corpo do MuJoCo.
        # Domínio duplicado = dois brains no mesmo grafo DDS, que é justamente o
        # que os domínios separados existem para evitar (os tópicos do brain são
        # absolutos e ignoram namespace).
        if slot in seen_slots:
            raise ValueError(f"{path}: slot {slot} duplicado")
        if domain in seen_domains:
            raise ValueError(f"{path}: domain_id {domain} duplicado")
        seen_slots.add(slot)
        seen_domains.add(domain)

        robots.append(RobotConfig(
            name=entry["name"],
            slot=slot,
            team=team,
            team_id=team_ids[team],
            player_id=int(entry["player_id"]),
            role=entry.get("role", "striker"),
            domain_id=domain,
        ))

    robots.sort(key=lambda r: r.slot)
    return MatchConfig(
        home_team_id=team_ids["home"],
        away_team_id=team_ids["away"],
        robots=tuple(robots),
    )
