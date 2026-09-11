"""Configuração do chute por força, independente da marcha."""
from dataclasses import dataclass
import math
import yaml
from hl_sim import paths


@dataclass(frozen=True)
class KickConfig:
    force_newtons: float = 10.0
    duration_seconds: float = 0.1
    distance_m: float = 0.45
    max_angle_degrees: float = 60.0
    max_ball_height_m: float = 0.3

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"kick.{name} deve ser finito e positivo")
        if self.max_angle_degrees > 180:
            raise ValueError("kick.max_angle_degrees deve ser <= 180")

    @classmethod
    def load(cls):
        with (paths.CONFIG / "simulation.yaml").open() as stream:
            return cls(**yaml.safe_load(stream)["kick"])
