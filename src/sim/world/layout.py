"""
hl_sim/sim/layout.py — resolve os índices de qpos/qvel/ctrl/sensor da cena.

Antes o SimBridge carregava isto como constante mágica:

    BALL_BODY_ID = 3
    _BODIES_BEFORE_ROBOTS = 4
    _BODIES_PER_ROBOT = 24
    _QPOS_PER_ROBOT = 30

Qualquer mexida em scenes/soccer_scene.xml (um gol a mais, um geom a menos)
deslocava tudo silenciosamente — o robô 4 passava a dirigir o corpo do robô 5.
Aqui os índices saem do próprio modelo, por nome, e a estrutura assumida é
verificada na carga: se a cena mudar de um jeito incompatível, estoura na hora
com mensagem clara em vez de simular errado.

Convenção de nomes da cena (assets/robotN/robotN_body.xml):
    robot1 → sem sufixo   ("Trunk",   "world_joint",   "AAHead_yaw",   "imu")
    robotN → sufixo "_N"  ("Trunk_3", "world_joint_3", "AAHead_yaw_3", "imu_3")

Este módulo não importa rclpy.
"""
from __future__ import annotations

import re
from dataclasses import dataclass

import mujoco

# Ordem dos <include> em scenes/soccer_scene.xml.  O índice nesta tupla é o
# `slot` usado em config/match.yaml e o índice de qpos/ctrl.
SCENE_ROBOT_ORDER: tuple[int, ...] = (1, 3, 5, 2, 4, 6)

NUM_ROBOTS       = len(SCENE_ROBOT_ORDER)
JOINTS_PER_ROBOT = 23
FIRST_JOINT      = "AAHead_yaw"   # primeiro atuador/junta de cada robô
BALL_BODY        = "ball"
BALL_JOINT       = "ball_free"

_SUFFIXED = re.compile(r"_\d+$")


def suffix_for(number: int) -> str:
    """robot1 não tem sufixo; os demais usam '_N'."""
    return "" if number == 1 else f"_{number}"


@dataclass(frozen=True)
class RobotLayout:
    number: int          # 1..6, o número do robô na cena
    slot:   int          # posição em SCENE_ROBOT_ORDER
    suffix: str

    trunk_body_id:  int  # body do Trunk, para xpos/xquat e xfrc_applied
    trunk_qpos_adr: int  # início dos 7 do free joint (pos 3 + quat 4)
    trunk_dof_adr:  int  # início dos 6 do free joint (lin 3 + ang 3)

    joint_qpos_adr: int  # início dos 23 qpos de junta
    joint_dof_adr:  int  # início dos 23 qvel de junta
    ctrl_adr:       int  # início dos 23 ctrl

    quat_sensor_adr: int  # framequat, 4 valores
    gyro_sensor_adr: int  # gyro, 3 valores

    @property
    def ctrl_slice(self) -> slice:
        return slice(self.ctrl_adr, self.ctrl_adr + JOINTS_PER_ROBOT)

    @property
    def joint_qpos_slice(self) -> slice:
        return slice(self.joint_qpos_adr, self.joint_qpos_adr + JOINTS_PER_ROBOT)

    @property
    def joint_dof_slice(self) -> slice:
        return slice(self.joint_dof_adr, self.joint_dof_adr + JOINTS_PER_ROBOT)


@dataclass(frozen=True)
class SceneLayout:
    ball_body_id:  int
    ball_qpos_adr: int
    ball_dof_adr:  int
    robots: tuple[RobotLayout, ...]

    def __len__(self) -> int:
        return len(self.robots)

    def by_number(self, number: int) -> RobotLayout:
        for r in self.robots:
            if r.number == number:
                return r
        raise KeyError(f"robot{number} não existe na cena")


def _id(model, objtype: int, name: str, what: str) -> int:
    idx = mujoco.mj_name2id(model, objtype, name)
    if idx < 0:
        raise ValueError(
            f"{what} '{name}' não existe na cena carregada.\n"
            f"  A cena mudou de nomes? Ver hl_sim/sim/layout.py e "
            f"scenes/soccer_scene.xml."
        )
    return idx


def resolve(model) -> SceneLayout:
    """Deriva todos os índices do modelo MuJoCo já carregado."""
    ball_body  = _id(model, mujoco.mjtObj.mjOBJ_BODY,  BALL_BODY,  "body da bola")
    ball_joint = _id(model, mujoco.mjtObj.mjOBJ_JOINT, BALL_JOINT, "free joint da bola")

    robots = []
    for slot, number in enumerate(SCENE_ROBOT_ORDER):
        s = suffix_for(number)

        trunk_body = _id(model, mujoco.mjtObj.mjOBJ_BODY,  f"Trunk{s}",       "body do tronco")
        free_joint = _id(model, mujoco.mjtObj.mjOBJ_JOINT, f"world_joint{s}", "free joint do tronco")
        first_jnt  = _id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{FIRST_JOINT}{s}", "primeira junta")
        first_act  = _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{FIRST_JOINT}{s}", "primeiro atuador")
        quat_sens  = _id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"orientation{s}",      "sensor framequat")
        gyro_sens  = _id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"angular-velocity{s}", "sensor gyro")

        # Os 23 atuadores de um robô precisam ser contíguos — é assim que
        # `data.ctrl[layout.ctrl_slice] = alvos` funciona.  Se alguém intercalar
        # os <include> de actuators, isto acusa em vez de escrever no robô errado.
        end = first_act + JOINTS_PER_ROBOT
        if end > model.nu:
            raise ValueError(
                f"robot{number}: atuadores {first_act}..{end} passam de nu={model.nu}"
            )
        for a in range(first_act, end):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or ""
            # robot1 não tem sufixo, então o teste dele é o inverso: o nome não
            # pode terminar em _N (isso seria um atuador de outro robô).
            ok = name.endswith(s) if s else not _SUFFIXED.search(name)
            if not ok:
                raise ValueError(
                    f"robot{number}: atuadores não são contíguos — índice {a} é "
                    f"'{name}', esperado sufixo {s!r}.  Confira a ordem dos "
                    f"<include> de actuators em scenes/soccer_scene.xml."
                )

        robots.append(RobotLayout(
            number=number,
            slot=slot,
            suffix=s,
            trunk_body_id=trunk_body,
            trunk_qpos_adr=int(model.jnt_qposadr[free_joint]),
            trunk_dof_adr=int(model.jnt_dofadr[free_joint]),
            joint_qpos_adr=int(model.jnt_qposadr[first_jnt]),
            joint_dof_adr=int(model.jnt_dofadr[first_jnt]),
            ctrl_adr=first_act,
            quat_sensor_adr=int(model.sensor_adr[quat_sens]),
            gyro_sensor_adr=int(model.sensor_adr[gyro_sens]),
        ))

    if model.nu != NUM_ROBOTS * JOINTS_PER_ROBOT:
        raise ValueError(
            f"cena tem nu={model.nu}, esperado "
            f"{NUM_ROBOTS}×{JOINTS_PER_ROBOT}={NUM_ROBOTS * JOINTS_PER_ROBOT}"
        )

    return SceneLayout(
        ball_body_id=ball_body,
        ball_qpos_adr=int(model.jnt_qposadr[ball_joint]),
        ball_dof_adr=int(model.jnt_dofadr[ball_joint]),
        robots=tuple(robots),
    )
