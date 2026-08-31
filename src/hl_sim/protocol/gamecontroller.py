"""
hl_sim/protocol/gamecontroller.py — protocolo RoboCupGameControlData v20 (HL v7).

Fonte única da verdade do canal árbitro → robôs.  Espelha
`src/game_controller/include/RoboCupGameControlData.h` do hsl-player.

Antes isto vivia em dois lugares: as constantes em `agent_interface.py` e o parse
binário no `gc_sniffer.py`.  Duas verdades para o mesmo struct de 158 bytes é
exatamente o tipo de divergência que só aparece em dia de jogo.

Quem usa:
  • hl_sim.ros2.game_controller_link  — monta GameState a partir da msg ROS2
  • hl_sim.cli.gc_sniffer             — decodifica o UDP cru para diagnóstico
  • hl_sim.agents.base                — reexporta as constantes para os agentes

NÃO existe mais `secondary_state`: o sub-estado vem de `game_phase` + `set_play`.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field

# ── portas ────────────────────────────────────────────────────────────────────
DATA_PORT   = 3838   # árbitro → robôs, broadcast
RETURN_PORT = 3939   # robô → árbitro, unicast (RGrt v4, enviado pelo brain)

# ── header do pacote ──────────────────────────────────────────────────────────
HEADER          = b"RGme"
STRUCT_VERSION  = 20
MAX_NUM_PLAYERS = 20   # tamanho do array no struct do fio

# Número de jogadores que o brain rastreia internamente (brain.cpp:1419).
# Enviar menos que isso estoura o array do brain.
HL_MAX_NUM_PLAYERS = 11

# ── estados ───────────────────────────────────────────────────────────────────
STATE_INITIAL  = 0
STATE_READY    = 1
STATE_SET      = 2
STATE_PLAYING  = 3
STATE_FINISHED = 4

GAME_PHASE_NORMAL            = 0
GAME_PHASE_PENALTY_SHOOT_OUT = 1
GAME_PHASE_EXTRA_TIME        = 2
GAME_PHASE_TIMEOUT           = 3

SET_PLAY_NONE               = 0
SET_PLAY_DIRECT_FREE_KICK   = 1
SET_PLAY_INDIRECT_FREE_KICK = 2
SET_PLAY_PENALTY_KICK       = 3
SET_PLAY_THROW_IN           = 4
SET_PLAY_GOAL_KICK          = 5
SET_PLAY_CORNER_KICK        = 6

KICKING_TEAM_NONE = 255

PENALTY_NONE       = 0
PENALTY_SENT_OFF   = 12
PENALTY_SUBSTITUTE = 13

# ── rótulos legíveis (diagnóstico) ────────────────────────────────────────────
STATES = {
    STATE_INITIAL: "INITIAL", STATE_READY: "READY", STATE_SET: "SET",
    STATE_PLAYING: "PLAYING", STATE_FINISHED: "FINISHED",
}
PHASES = {
    GAME_PHASE_NORMAL: "NORMAL", GAME_PHASE_PENALTY_SHOOT_OUT: "PENALTY_SHOOT_OUT",
    GAME_PHASE_EXTRA_TIME: "EXTRA_TIME", GAME_PHASE_TIMEOUT: "TIMEOUT",
}
SET_PLAYS = {
    SET_PLAY_NONE: "NONE", SET_PLAY_DIRECT_FREE_KICK: "DIRECT_FREE_KICK",
    SET_PLAY_INDIRECT_FREE_KICK: "INDIRECT_FREE_KICK",
    SET_PLAY_PENALTY_KICK: "PENALTY_KICK", SET_PLAY_THROW_IN: "THROW_IN",
    SET_PLAY_GOAL_KICK: "GOAL_KICK", SET_PLAY_CORNER_KICK: "CORNER_KICK",
}
PENALTIES = {
    0: "NONE", 1: "ILLEGAL_POSITIONING", 2: "MOTION_IN_SET", 3: "MOTION_IN_STOP",
    4: "LOCAL_GAME_STUCK", 5: "INCAPABLE_ROBOT", 6: "PICK_UP", 7: "BALL_HOLDING",
    8: "LEAVING_THE_FIELD", 9: "PLAYING_WITH_ARMS_HANDS", 10: "PUSHING",
    11: "CAUTIONED", 12: "SENT_OFF", 13: "SUBSTITUTE",
}

# ── layout binário ────────────────────────────────────────────────────────────
# char[4] + 10×uint8 + 2×int16
_HEAD_FMT = "<4s10Bhh"
# uint8×6 + uint16×2 + RobotInfo[20] (3 bytes cada)
_TEAM_FMT = f"<6B2H{MAX_NUM_PLAYERS * 3}s"

_HEAD_SIZE  = struct.calcsize(_HEAD_FMT)    # 18
_TEAM_SIZE  = struct.calcsize(_TEAM_FMT)    # 70
PACKET_SIZE = _HEAD_SIZE + 2 * _TEAM_SIZE   # 158

# O game_controller_node descarta em silêncio tudo que não bater com estes três
# (game_controller_node.cpp:105-118).
def is_acceptable(data: bytes) -> tuple[bool, str]:
    """(aceitável?, motivo da recusa) — mesma checagem do game_controller_node."""
    if len(data) != PACKET_SIZE:
        return False, f"tamanho {len(data)} != {PACKET_SIZE}"
    if data[:4] != HEADER:
        return False, f"header {data[:4]!r} != {HEADER!r}"
    if data[4] != STRUCT_VERSION:
        return False, f"version {data[4]} != {STRUCT_VERSION}"
    return True, ""


def _decode_team(raw: bytes) -> dict:
    (team_number, field_colour, gk_colour, goalkeeper, score, penalty_shot,
     single_shots, message_budget, players_raw) = struct.unpack(_TEAM_FMT, raw)
    players = [
        {
            "penalty":   players_raw[i * 3],
            "secs_till": players_raw[i * 3 + 1],
            "cautions":  players_raw[i * 3 + 2],
        }
        for i in range(MAX_NUM_PLAYERS)
    ]
    return {
        "team_number": team_number, "field_colour": field_colour,
        "gk_colour": gk_colour, "goalkeeper": goalkeeper, "score": score,
        "penalty_shot": penalty_shot, "single_shots": single_shots,
        "message_budget": message_budget, "players": players,
    }


def decode(data: bytes) -> dict:
    """Decodifica um RoboCupGameControlData v20.  Lança struct.error se malformado."""
    (header, version, packet_number, players_per_team, competition_type, stopped,
     game_phase, state, set_play, first_half, kicking_team,
     secs_remaining, secondary_time) = struct.unpack(_HEAD_FMT, data[:_HEAD_SIZE])

    teams = [
        _decode_team(data[_HEAD_SIZE + i * _TEAM_SIZE:
                          _HEAD_SIZE + (i + 1) * _TEAM_SIZE])
        for i in range(2)
    ]
    return {
        "header": header, "version": version, "packet_number": packet_number,
        "players_per_team": players_per_team, "competition_type": competition_type,
        "stopped": stopped, "game_phase": game_phase, "state": state,
        "set_play": set_play, "first_half": first_half,
        "kicking_team": kicking_team, "secs_remaining": secs_remaining,
        "secondary_time": secondary_time, "teams": teams,
    }


# ── estado de jogo resolvido para um time ─────────────────────────────────────

@dataclass(frozen=True)
class GameState:
    """
    Recorte do GameControlData v20 já resolvido para o ponto de vista de um time.

    Imutável de propósito: o callback do ROS2 cria um novo e troca a referência,
    então o loop de controle nunca lê um estado meio atualizado.
    """
    state:          int = STATE_INITIAL
    game_phase:     int = GAME_PHASE_NORMAL
    set_play:       int = SET_PLAY_NONE
    stopped:        bool = False
    secs_remaining: int = 0
    secondary_time: int = 0
    kicking_team:   int = KICKING_TEAM_NONE
    my_score:       int = 0
    oppo_score:     int = 0
    # penalty por jogador do meu time, índice 0-based (player_id - 1)
    penalties:      tuple = field(default_factory=tuple)
    packet_number:  int = 0
    received:       bool = False   # False = nunca chegou pacote do GameController

    def is_playing(self) -> bool:
        return self.state == STATE_PLAYING and not self.stopped

    def is_penalized(self, player_id: int) -> bool:
        """player_id é 1-based, como no protocolo e no config.yaml do brain."""
        idx = player_id - 1
        if idx < 0 or idx >= len(self.penalties):
            return False
        return self.penalties[idx] != PENALTY_NONE

    def is_kickoff_side(self, team_id: int) -> bool:
        return self.kicking_team == team_id

    def describe(self) -> str:
        return (f"{STATES.get(self.state, self.state)}"
                f"/{PHASES.get(self.game_phase, self.game_phase)}"
                f"/{SET_PLAYS.get(self.set_play, self.set_play)}"
                f"{' STOPPED' if self.stopped else ''}")
