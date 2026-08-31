"""
hl_sim/paths.py — resolve os caminhos do repositório uma única vez.

Antes cada módulo fazia `Path(__file__).parent.parent / "scenes"`, o que quebrou
assim que os arquivos mudaram de profundidade na árvore.  Aqui o cálculo é feito
num lugar só, e `HL_SIM_ROOT` permite sobrescrever (container, testes).
"""
from __future__ import annotations

import os
from pathlib import Path

# src/hl_sim/paths.py → src/hl_sim → src → raiz do repositório
_DEFAULT_ROOT = Path(__file__).resolve().parents[2]

ROOT: Path = Path(os.environ.get("HL_SIM_ROOT", _DEFAULT_ROOT)).resolve()

ASSETS: Path = ROOT / "assets"
SCENES: Path = ROOT / "scenes"
MODELS: Path = ROOT / "models"
CONFIG: Path = ROOT / "config"

DEFAULT_SCENE: Path = SCENES / "soccer_scene.xml"
MATCH_CONFIG:  Path = CONFIG / "match.yaml"


def require(path: Path, what: str) -> Path:
    """Falha cedo e com mensagem útil em vez de estourar dentro do MuJoCo."""
    if not path.exists():
        raise FileNotFoundError(
            f"{what} não encontrado: {path}\n"
            f"  ROOT resolvido para: {ROOT}\n"
            f"  Defina HL_SIM_ROOT se o repositório estiver noutro lugar."
        )
    return path
