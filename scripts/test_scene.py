#!/usr/bin/env python3
"""
test_scene.py — Fase 1: testes de sanidade do soccer_scene.xml

Uso:
    uv run python scripts/test_scene.py
    uv run python scripts/test_scene.py --viewer
    uv run test-scene --viewer
"""
import sys
import argparse
import numpy as np
from pathlib import Path

try:
    import mujoco
except ImportError:
    print("ERRO: mujoco não instalado.  Execute: uv sync")
    sys.exit(1)

SCENE_PATH = Path(__file__).parent.parent / "scenes" / "soccer_scene.xml"

# ── terminal colors ──────────────────────────────────────────────────────────
BOLD   = "\033[1m"
CYAN   = "\033[96m"
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
DIM    = "\033[2m"
RESET  = "\033[0m"

def _ok(msg):   print(f"  {GREEN}✓{RESET} {msg}")
def _fail(msg): print(f"  {RED}✗{RESET} {msg}")
def _info(msg): print(f"  {DIM}·{RESET} {msg}")
def _section(title): print(f"\n{BOLD}{CYAN}── {title}{RESET}")


# ── helpers ──────────────────────────────────────────────────────────────────

def _body_id(m: mujoco.MjModel, name: str) -> int:
    return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)


def _make_data(m: mujoco.MjModel, keyframe: int = 0) -> mujoco.MjData:
    """Cria MjData resetado para um keyframe."""
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, keyframe)
    mujoco.mj_forward(m, d)
    return d


# ── testes individuais ───────────────────────────────────────────────────────

def test_load() -> tuple[mujoco.MjModel | None, bool]:
    """Carrega o modelo e verifica especificação numérica."""
    _section("1. Carregamento do modelo")

    if not SCENE_PATH.exists():
        _fail(f"Arquivo não encontrado: {SCENE_PATH}")
        return None, False

    try:
        m = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    except Exception as exc:
        _fail(f"Erro ao carregar XML: {exc}")
        return None, False

    # nq = ball_free(7) + world_joint(7) + 23 joints = 37
    # nv = 6 + 6 + 23 = 35
    # nu = 23 atuadores de posição
    # nbody = world(1) + goal×2 + ball + Trunk + 24 corpos T1 = 28
    expected = {"nq": 37, "nv": 35, "nu": 23, "nbody": 28}
    passed = True
    for attr, exp in expected.items():
        val = getattr(m, attr)
        if val == exp:
            _ok(f"{attr} = {val}")
        else:
            _fail(f"{attr} = {val}  (esperado {exp})")
            passed = False

    return m, passed


def print_model_summary(m: mujoco.MjModel) -> None:
    _section("Resumo do modelo")

    joint_names = [m.joint(i).name for i in range(m.njnt)]
    body_names  = [m.body(i).name  for i in range(m.nbody)]
    act_names   = [m.actuator(i).name for i in range(m.nu)]

    _info(f"Joints  ({m.njnt}): {joint_names[:4]} … [{m.njnt - 4} mais]")
    _info(f"Bodies  ({m.nbody}): {body_names[:6]} … [{m.nbody - 6} mais]")
    _info(f"Atuadores ({m.nu}): {act_names[:4]} … [{m.nu - 4} mais]")
    _info(f"Timestep : {m.opt.timestep} s  |  Gravidade: {m.opt.gravity}")


def test_keyframe(m: mujoco.MjModel) -> bool:
    """Valida posições após reset para keyframe 'home'."""
    _section("2. Keyframe 'home'")

    d = _make_data(m, keyframe=0)
    passed = True

    # Trunk: deve estar > 0.5 m (de pé)
    trunk_z = d.xpos[_body_id(m, "Trunk")][2]
    if trunk_z > 0.5:
        _ok(f"Trunk z = {trunk_z:.3f} m")
    else:
        _fail(f"Trunk z = {trunk_z:.3f} m  (esperado > 0.5)")
        passed = False

    # Bola: centro a ~0.11 m do chão
    ball_z = d.xpos[_body_id(m, "ball")][2]
    if abs(ball_z - 0.11) < 0.03:
        _ok(f"Ball z = {ball_z:.3f} m  (≈ 0.11 m)")
    else:
        _fail(f"Ball z = {ball_z:.3f} m  (esperado ≈ 0.11)")
        passed = False

    # Pés perto do chão — left_foot_link e right_foot_link
    for foot in ("left_foot_link", "right_foot_link"):
        fz = d.xpos[_body_id(m, foot)][2]
        if fz < 0.15:
            _ok(f"{foot} z = {fz:.3f} m  (próximo ao chão)")
        else:
            _fail(f"{foot} z = {fz:.3f} m  (pé suspenso?)")
            passed = False

    # Sem NaN
    if not np.any(np.isnan(d.qpos)):
        _ok("qpos sem NaN")
    else:
        _fail("NaN encontrado em qpos")
        passed = False

    return passed


def test_simulation(m: mujoco.MjModel, duration: float = 2.0) -> bool:
    """Simula N segundos com a posição home e verifica estabilidade.
    Nota: sem a locomotion policy (Fase 2) o robô assenta gradualmente —
    verificamos apenas que não cai abruptamente nem gera NaN.
    """
    _section(f"3. Estabilidade da simulação ({duration} s)")

    d = _make_data(m, keyframe=0)
    steps = int(duration / m.opt.timestep)
    trunk_id = _body_id(m, "Trunk")

    min_trunk_z = float("inf")
    nan_at: int | None = None
    fall_at: int | None = None

    for i in range(steps):
        mujoco.mj_step(m, d)

        if np.any(np.isnan(d.qpos)) or np.any(np.isnan(d.qvel)):
            nan_at = i
            break

        tz = d.xpos[trunk_id][2]
        min_trunk_z = min(min_trunk_z, tz)
        if tz < 0.15 and fall_at is None:
            fall_at = i

    passed = True

    if nan_at is not None:
        _fail(f"NaN no passo {nan_at}  (t = {nan_at * m.opt.timestep:.3f} s)")
        passed = False
    else:
        _ok(f"Sem NaN em {steps} passos")

    trunk_z_final = d.xpos[trunk_id][2]
    # Sem policy de balanço o robô assenta — threshold relaxado para Fase 1
    if trunk_z_final > 0.2:
        _ok(f"Trunk z final = {trunk_z_final:.3f} m  (robô em pé ou assentando)")
    else:
        _fail(f"Trunk z final = {trunk_z_final:.3f} m  (colapso — verificar física)")
        passed = False

    if fall_at is not None:
        _info(f"Colapso detectado em t = {fall_at * m.opt.timestep:.2f} s  (Trunk z < 0.15)")

    _info(f"Trunk z mínimo = {min_trunk_z:.3f} m  |  tempo simulado = {steps * m.opt.timestep:.1f} s")

    return passed


def test_ball_physics(m: mujoco.MjModel) -> bool:
    """Verifica que a bola rola ao receber impulso e retorna ao chão."""
    _section("4. Física da bola")

    d = _make_data(m, keyframe=0)
    ball_id = _body_id(m, "ball")
    x0 = d.xpos[ball_id][0]

    # 100 N por 0.1 s → impulso = 10 N·s; bola de 450 g → Δv ≈ 22 m/s
    force_steps = int(0.1 / m.opt.timestep)
    for _ in range(force_steps):
        d.xfrc_applied[ball_id] = [100.0, 0, 0, 0, 0, 0]
        mujoco.mj_step(m, d)
    d.xfrc_applied[ball_id] = [0] * 6

    for _ in range(int(1.0 / m.opt.timestep)):   # rola por 1 s
        mujoco.mj_step(m, d)

    passed = True
    dx = d.xpos[ball_id][0] - x0
    if dx > 1.0:
        _ok(f"Bola percorreu {dx:.2f} m após impulso")
    else:
        _fail(f"Bola mal se moveu: Δx = {dx:.2f} m")
        passed = False

    ball_z = d.xpos[ball_id][2]
    if ball_z < 0.25:
        _ok(f"Bola permanece no chão  (z = {ball_z:.3f} m)")
    else:
        _fail(f"Bola flutuando  (z = {ball_z:.3f} m)")
        passed = False

    return passed


def run_viewer(m: mujoco.MjModel) -> None:
    try:
        import mujoco.viewer
        import time
    except ImportError:
        print(f"  {YELLOW}mujoco.viewer não disponível neste ambiente{RESET}")
        return

    _section("Viewer interativo")
    print("  Abrindo viewer…  (feche a janela para encerrar)\n")

    d = _make_data(m, keyframe=0)
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.distance = 12.0
        v.cam.elevation = -20
        while v.is_running():
            mujoco.mj_step(m, d)
            v.sync()
            time.sleep(m.opt.timestep)


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Testa soccer_scene.xml — Fase 1 do t1_soccer_sim"
    )
    parser.add_argument(
        "--viewer", action="store_true",
        help="Abre o viewer interativo após os testes"
    )
    args = parser.parse_args()

    print(f"\n{BOLD}T1 Soccer Sim — Teste da Fase 1{RESET}")
    print(f"{DIM}cena: {SCENE_PATH}{RESET}")

    results: list[tuple[str, bool]] = []

    # 1. carregamento
    m, r = test_load()
    results.append(("Carregamento do modelo", r))
    if m is None:
        print(f"\n{RED}{BOLD}Não foi possível carregar o modelo. Abortando.{RESET}\n")
        sys.exit(1)

    print_model_summary(m)

    # 2–4. testes
    results.append(("Keyframe home",          test_keyframe(m)))
    results.append(("Estabilidade 4 s",       test_simulation(m)))
    results.append(("Física da bola",         test_ball_physics(m)))

    # resumo final
    _section("Resultado")
    all_pass = True
    for name, ok in results:
        tag = f"{GREEN}PASS{RESET}" if ok else f"{RED}FAIL{RESET}"
        print(f"  {tag}  {name}")
        if not ok:
            all_pass = False

    if all_pass:
        print(f"\n{GREEN}{BOLD}  Fase 1 completa ✓{RESET}\n")
    else:
        print(f"\n{RED}{BOLD}  Fase 1 com falhas ✗{RESET}\n")

    if args.viewer:
        run_viewer(m)

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
