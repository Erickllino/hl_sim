# coding: utf-8
"""
docker/launch/robot.launch.py — sobe um robô inteiro num container.

No robô real, brain_node e game_controller_node rodam na mesma máquina, no mesmo
grafo ROS2.  Aqui é igual: um container = um robô = os dois nós no mesmo
ROS_DOMAIN_ID.

Não reimplementa nada — dá include nos launch files do próprio hsl-player, para
não divergir deles quando o brain mudar.

    ros2 launch /etc/hl/robot.launch.py
    ros2 launch /etc/hl/robot.launch.py tree:=game.xml disable_log:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gc_launch = PathJoinSubstitution(
        [FindPackageShare("game_controller"), "launch", "launch.py"]
    )
    brain_launch = PathJoinSubstitution(
        [FindPackageShare("brain"), "launch", "launch.py"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("tree", default_value="game.xml",
                              description="behavior tree carregada pelo brain"),
        # sim:=false de propósito.  sim:=true liga use_sim_time e o simulador
        # ainda não publica /clock — o brain ficaria esperando um relógio que
        # nunca chega.  Ver docs/gamecontroller-integracao.md.
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("disable_log", default_value="false"),
        DeclareLaunchArgument("disable_com", default_value="false"),

        # Ponte UDP 3838 → /robocup/game_controller.  Um por container: cada robô
        # tem sua própria network namespace, então não há disputa pela porta
        # 3838 (o game_controller_node faz bind sem SO_REUSEADDR).
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gc_launch)),

        # O brain lê team_id/player_id/player_role do config_local.yaml que o
        # entrypoint instalou a partir de config/match.yaml.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(brain_launch),
            launch_arguments={
                "tree":        LaunchConfiguration("tree"),
                "sim":         LaunchConfiguration("sim"),
                "disable_log": LaunchConfiguration("disable_log"),
                "disable_com": LaunchConfiguration("disable_com"),
            }.items(),
        ),
    ])
