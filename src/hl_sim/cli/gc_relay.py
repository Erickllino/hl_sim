#!/usr/bin/env python3
"""
hl_sim/cli/gc_relay.py — reemite o broadcast do GameController na rede do Docker.

Por que isto existe
-------------------
O GameController roda no host (é um app Tauri nativo, como o notebook do árbitro
na competição) e emite broadcast UDP na 3838.  Um broadcast para 255.255.255.255
sai pela interface da rota padrão, e a bridge do Docker não é ela — então o
pacote pode simplesmente não chegar nos containers dos robôs.

Primeiro tente apontar o endereço de broadcast do próprio GameController para a
subnet do Docker.  Este relay é o plano B.

Ele NÃO interpreta nada: repassa os bytes exatamente como chegaram, para o
parse do game_controller_node continuar sendo o parse de verdade.  A validação
aqui é só diagnóstico.

Uso (precisa da rede do host para ver o broadcast original):
    python -m hl_sim.cli.gc_relay --subnet 172.28.0.0/16

    docker compose -f docker/compose.yaml --profile relay up gc-relay
"""
from __future__ import annotations

import argparse
import ipaddress
import socket
import sys
import time

from hl_sim.protocol.gamecontroller import DATA_PORT, PACKET_SIZE, is_acceptable

DIM = "\033[2m"; GREEN = "\033[92m"; RED = "\033[91m"; RESET = "\033[0m"


# Subnet da rede hl_net, como declarada em docker/compose.yaml.
DEFAULT_SUBNET = "172.28.0.0/16"


def main() -> None:
    ap = argparse.ArgumentParser(
        description="repassa o broadcast do GameController para a subnet do Docker"
    )
    ap.add_argument("--subnet", default=DEFAULT_SUBNET,
                    help=f"subnet da rede do Docker; define o endereço de destino "
                         f"e quais origens são o próprio eco  [default: {DEFAULT_SUBNET}]")
    ap.add_argument("--to", default=None,
                    help="endereço de destino, se não for o broadcast da subnet")
    ap.add_argument("--port", type=int, default=DATA_PORT,
                    help=f"porta de escuta e de destino  [default: {DATA_PORT}]")
    ap.add_argument("--quiet", action="store_true",
                    help="só imprime o resumo periódico")
    args = ap.parse_args()

    # Nada de adivinhar prefixo a partir do endereço de broadcast: 10.255.255.255
    # é broadcast válido de /8 e de /16, e errar aqui faz o relay ou entrar em
    # loop consigo mesmo ou descartar pacotes bons.  A subnet vem explícita.
    try:
        target_net = ipaddress.IPv4Network(args.subnet, strict=False)
    except ValueError as e:
        raise SystemExit(f"--subnet: {e}")
    target_addr = args.to or str(target_net.broadcast_address)

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # SO_REUSEADDR para conviver com um game_controller_node ou um sniffer que já
    # esteja escutando a 3838 no host: em broadcast, todos os sockets ligados à
    # porta recebem sua cópia.
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("", args.port))
    rx.settimeout(1.0)

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print(f"· escutando  0.0.0.0:{args.port}")
    print(f"· reemitindo {target_addr}:{args.port}  (subnet {target_net})")
    print(f"{DIM}· pacotes vindos de {target_net} são ignorados — seriam o próprio eco{RESET}")

    relayed = dropped = rejected = 0
    last_report = time.time()

    try:
        while True:
            try:
                data, (src_ip, _) = rx.recvfrom(4096)
            except socket.timeout:
                data = None

            if data is not None:
                # A reemissão sai com IP de origem dentro da subnet de destino
                # (o gateway da bridge).  Sem este teste o relay entraria em loop
                # consigo mesmo.
                if ipaddress.IPv4Address(src_ip) in target_net:
                    dropped += 1
                else:
                    ok, why = is_acceptable(data)
                    if not ok:
                        rejected += 1
                        if not args.quiet:
                            print(f"  {RED}✗{RESET} de {src_ip}: {why} "
                                  f"{DIM}(o game_controller_node descartaria){RESET}")
                    tx.sendto(data, (target_addr, args.port))
                    relayed += 1
                    if not args.quiet and relayed == 1:
                        print(f"  {GREEN}✓{RESET} primeiro pacote de {src_ip}, "
                              f"{len(data)} bytes (esperado {PACKET_SIZE})")

            now = time.time()
            if now - last_report >= 10.0:
                print(f"{DIM}· {relayed} repassados · {rejected} malformados · "
                      f"{dropped} ecos ignorados{RESET}")
                last_report = now
                if relayed == 0:
                    print(f"  {RED}nenhum pacote em 10 s{RESET} — o GameController "
                          f"está emitindo? Confira com: python -m hl_sim.cli.gc_sniffer",
                          file=sys.stderr)
    except KeyboardInterrupt:
        print(f"\n· fim: {relayed} repassados, {rejected} malformados, "
              f"{dropped} ecos ignorados")
    finally:
        rx.close()
        tx.close()


if __name__ == "__main__":
    main()
