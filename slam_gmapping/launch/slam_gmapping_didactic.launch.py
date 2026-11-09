# Uso:
#   ros2 launch slam_gmapping_didactic slam_gmapping_didactic.launch.py                 (preset didattico)
#   ros2 launch slam_gmapping_didactic slam_gmapping_didactic.launch.py config:=originale
#   ros2 launch slam_gmapping_didactic slam_gmapping_didactic.launch.py rviz:=true      (apre anche RViz2
#                                                                     con la configurazione didattica)
# I parametri dei file YAML si trovano in config/<nome>.yaml del pacchetto.
#
# Se nel file di configurazione odom_drift e' true, viene avviato anche il nodo
# odom_drift.py, che aggiunge una deriva artificiale all'odometria; i suoi parametri
# (drift_*) si leggono dallo stesso file, nella sezione di slam_gmapping_didactic.
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def nodi(context):
    share = get_package_share_directory('slam_gmapping_didactic')
    config = LaunchConfiguration('config').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params_file = os.path.join(share, 'config', config + '.yaml')
    if not os.path.isfile(params_file):
        raise RuntimeError(f'File di configurazione non trovato: {params_file}')

    with open(params_file) as f:
        params = (yaml.safe_load(f) or {}).get('slam_gmapping_didactic', {}).get('ros__parameters', {})

    azioni = [
        Node(
            package='slam_gmapping_didactic',
            executable='slam_gmapping_didactic',
            name='slam_gmapping_didactic',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ]
    if params.get('odom_drift', False):
        drift = {k: v for k, v in params.items() if k.startswith('drift_')}
        drift['use_sim_time'] = use_sim_time
        azioni.append(LogInfo(msg='Deriva dell\'odometria ATTIVA (odom_drift: true)'))
        azioni.append(Node(
            package='slam_gmapping_didactic',
            executable='odom_drift.py',
            name='odom_drift',
            output='screen',
            parameters=[drift],
        ))
    return azioni


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    rviz_file = PathJoinSubstitution([FindPackageShare('slam_gmapping_didactic'), 'rviz', 'didattico.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config', default_value='didattico',
                              description='Preset di parametri: nome di un file in config/ senza .yaml'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Apre RViz2 con la configurazione didattica'),
        OpaqueFunction(function=nodi),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_didattico',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
            output='log',
        ),
    ])
