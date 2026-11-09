# Avvia Gazebo con il TurtleBot3 Burger in uno dei mondi disponibili.
#
#   ros2 launch slam_gmapping_didactic gazebo_mondo.launch.py                      (mondo "anello")
#   ros2 launch slam_gmapping_didactic gazebo_mondo.launch.py mondo:=turtlebot3_world
#
# Mondi:
#   anello            corridoio ad anello (giro di circa 30 m) per la chiusura del ciclo;
#                     file worlds/anello.world di questo pacchetto
#   turtlebot3_world  il mondo standard del TurtleBot3 (lo stesso di start_simulation.sh)
#
# La posa iniziale del robot dipende dal mondo; si puo' cambiare con x:=, y:=, yaw:=.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# mondo -> (file, posa iniziale x, y, yaw)
MONDI = {
    'anello': (os.path.join(get_package_share_directory('slam_gmapping_didactic'),
                            'worlds', 'anello.world'), -4.75, -1.0, 1.5708),
    'turtlebot3_world': (os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                                      'worlds', 'turtlebot3_world.world'), -2.0, -0.5, 0.0),
}


def avvia(context):
    mondo = LaunchConfiguration('mondo').perform(context)
    if mondo not in MONDI:
        raise RuntimeError(f"Mondo '{mondo}' sconosciuto. Disponibili: {', '.join(MONDI)}")
    world, x0, y0, yaw0 = MONDI[mondo]
    x = LaunchConfiguration('x').perform(context) or str(x0)
    y = LaunchConfiguration('y').perform(context) or str(y0)
    yaw = LaunchConfiguration('yaw').perform(context) or str(yaw0)

    gazebo_ros = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_description = get_package_share_directory('turtlebot3_description')
    model = os.path.join(turtlebot3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf')
    urdf = os.path.join(turtlebot3_description, 'urdf', 'turtlebot3_burger.urdf')

    return [
        ExecuteProcess(
            cmd=['gzserver', '--verbose',
                 '-slibgazebo_ros_init.so',
                 '-slibgazebo_ros_factory.so',
                 '-slibgazebo_ros_force_system.so',
                 world],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'))
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf, ' namespace:=']),
                'use_sim_time': True,
            }],
        ),
        TimerAction(
            period=10.0,
            actions=[Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'burger', '-file', model,
                           '-x', x, '-y', y, '-z', '0.01', '-Y', yaw, '-timeout', '90'],
                output='screen',
            )],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mondo', default_value='anello',
                              description='Mondo Gazebo: anello | turtlebot3_world'),
        DeclareLaunchArgument('x', default_value='', description='Posa iniziale x (vuoto = default del mondo)'),
        DeclareLaunchArgument('y', default_value='', description='Posa iniziale y (vuoto = default del mondo)'),
        DeclareLaunchArgument('yaw', default_value='', description='Orientamento iniziale (rad)'),
        OpaqueFunction(function=avvia),
    ])
