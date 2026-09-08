#!/usr/bin/env python3
"""
hl_sim/cli/gc_sniffer.py — decodifica os pacotes UDP do GameController na porta 3838.

Ferramenta de diagnóstico do canal árbitro → robôs.  Python puro: não precisa de
ROS2, MuJoCo nem do workspace do hsl-player buildado.

Para que serve
--------------
O game_controller_node do hsl-player descarta silenciosamente todo pacote que não
tenha exatamente 158 bytes e version == 20 (game_controller_node.cpp:105-118).
Quando o estado de jogo "não chega no brain", este script responde em 10 segundos
se o problema é o árbitro, a rede ou o ROS2.

Uso
---
    python3 hl_sim/cli/gc_sniffer.py
    python3 hl_sim/cli/gc_sniffer.py --port 3838 --all

Roda em paralelo com o game_controller_node sem roubar pacotes: 3838 recebe
broadcast, e todo socket com SO_REUSEADDR ligado à porta recebe sua cópia.

Nota sobre a porta 3939 (robô → árbitro): aquele canal é unicast, então um
sniffer ligado nele PODE roubar pacotes do GameController.  Para inspecionar o
retorno, use tcpdump:  sudo tcpdump -i any -X udp port 3939
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

# O protocolo mora em hl_sim/protocol/gamecontroller.py — módulo único, sem
# rclpy e sem MuJoCo, compartilhado com o GameControllerLink.  Este script só
# apresenta o que aquele módulo decodifica.
from hl_sim.protocol.gamecontroller import (
    DATA_PORT,
    HEADER,
    MAX_NUM_PLAYERS,
    PACKET_SIZE,
    PENALTIES,
    PHASES,
    SET_PLAYS,
    STATES,
    STRUCT_VERSION,
    decode,
    is_acceptable,
)

BOLD = "\033[1m"; GREEN = "\033[92m"; RED = "\033[91m"
YELLOW = "\033[93m"; DIM = "\033[2m"; RESET = "\033[0m"


def _summary(pkt: dict) -> str:
    t0, t1 = pkt["teams"]
    kick = pkt["kicking_team"]
    kick_s = "NONE" if kick == 255 else str(kick)
    return (
        f"{BOLD}{STATES.get(pkt['state'], pkt['state'])}{RESET}"
        f"  phase={PHASES.get(pkt['game_phase'], pkt['game_phase'])}"
        f"  set_play={SET_PLAYS.get(pkt['set_play'], pkt['set_play'])}"
        f"  stopped={pkt['stopped']}"
        f"  kicking_team={kick_s}"
        f"  placar={t0['team_number']}:{t0['score']} × {t1['team_number']}:{t1['score']}"
        f"  {DIM}secs={pkt['secs_remaining']} sec2={pkt['secondary_time']}{RESET}"
    )


def _penalty_lines(pkt: dict) -> list:
    lines = []
    for team in pkt["teams"]:
        punished = [
            f"p{i + 1}={PENALTIES.get(p['penalty'], p['penalty'])}"
            for i, p in enumerate(team["players"])
            if p["penalty"] != 0
        ]
        if punished:
            lines.append(f"    time {team['team_number']}: {' '.join(punished)}")
    return lines


def main() -> None:
    ap = argparse.ArgumentParser(description="Sniffer do GameController (UDP 3838)")
    ap.add_argument("--port", type=int, default=DATA_PORT)
    ap.add_argument("--all", action="store_true",
                    help="Imprime todo pacote, não só as mudanças de estado")
    ap.add_argument("--bind", default="0.0.0.0")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    try:
        sock.bind((args.bind, args.port))
    except OSError as exc:
        print(f"{RED}✗{RESET} não consegui abrir {args.bind}:{args.port} — {exc}")
        sys.exit(1)

    print(f"{BOLD}GameController sniffer{RESET}  {args.bind}:{args.port}")
    print(f"{DIM}esperando {PACKET_SIZE} bytes por pacote, "
          f"header={HEADER.decode()} version={STRUCT_VERSION}{RESET}")
    print(f"{DIM}Ctrl-C para sair{RESET}\n")

    last_summary = None
    count = 0
    t0 = time.time()

    try:
        while True:
            data, addr = sock.recvfrom(4096)
            count += 1
            src = f"{addr[0]}:{addr[1]}"

            if len(data) != PACKET_SIZE:
                print(f"{RED}✗{RESET} {src} tamanho {len(data)}, esperado "
                      f"{PACKET_SIZE} — o game_controller_node descartaria este pacote")
                continue

            try:
                pkt = decode(data)
            except struct.error as exc:
                print(f"{RED}✗{RESET} {src} pacote não decodificável: {exc}")
                continue

            if pkt["header"] != HEADER:
                print(f"{RED}✗{RESET} {src} header {pkt['header']!r}, "
                      f"esperado {HEADER!r}")
                continue

            if pkt["version"] != STRUCT_VERSION:
                print(f"{RED}✗{RESET} {src} version={pkt['version']}, o hsl-player "
                      f"só aceita {STRUCT_VERSION} — TODOS os pacotes serão descartados")
                continue

            summary = _summary(pkt)
            if args.all or summary != last_summary:
                stamp = time.strftime("%H:%M:%S")
                print(f"{GREEN}✓{RESET} {DIM}{stamp} {src} #{pkt['packet_number']}{RESET}  {summary}")
                for line in _penalty_lines(pkt):
                    print(f"{YELLOW}{line}{RESET}")
                last_summary = summary

    except KeyboardInterrupt:
        dt = time.time() - t0
        rate = count / dt if dt > 0 else 0.0
        print(f"\n{DIM}{count} pacotes em {dt:.1f}s ({rate:.1f}/s){RESET}")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
