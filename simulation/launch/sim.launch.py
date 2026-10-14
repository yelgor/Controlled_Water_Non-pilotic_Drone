"""Катер на воді: Gazebo Sim + ros_gz_bridge.

Запуск:
    ros2 launch ~/simulation/launch/sim.launch.py                 # GUI + шум + EKF + автопілот (квадрат) + логер
    ros2 launch ~/simulation/launch/sim.launch.py mode:=circle    # square circle figure8 zigzag fwd_back random steps
    ros2 launch ~/simulation/launch/sim.launch.py mode:=manual    # без автопілота, відкриває вікно teleop.py
    ros2 launch ~/simulation/launch/sim.launch.py gui:=false      # тільки сервер (без вікна)
    ros2 launch ~/simulation/launch/sim.launch.py auto:=false     # лише симуляція, жодних вузлів
    ros2 launch ~/simulation/launch/sim.launch.py use_gt:=true    # регулятор по ground truth, без KF
    ros2 launch ~/simulation/launch/sim.launch.py heading_kp:=5.0 # коефіцієнти PID без правки коду
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(ROOT, 'worlds', 'water.sdf')
BRIDGE_CONFIG = os.path.join(ROOT, 'config', 'bridge.yaml')
SCRIPTS = os.path.join(ROOT, 'scripts')


def script(name, *params):
    """Вузол-скрипт з scripts/ на часі симуляції."""
    args = ['-p', 'use_sim_time:=true']
    for p in params:
        args += ['-p', p]
    return ExecuteProcess(cmd=['python3', os.path.join(SCRIPTS, name), '--ros-args', *args],
                          output='screen')


def launch_gz(context):
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    gz_args = f'-r {WORLD}' if gui else f'-r -s {WORLD}'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )]


# Передаються в controller.py, лише якщо задані (усі, крім seed, - дробові числа)
CONTROLLER_ARGS = ['u_ref', 'size', 'seed', 'lookahead',
                   'speed_kp', 'speed_ki', 'speed_kd', 'heading_kp', 'heading_ki', 'heading_kd']


def teleop_window():
    """teleop.py читає клавіші з терміналу, а в ros2 launch його немає - відкриваємо окреме вікно."""
    cmd = (f'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", 0)}; '
           f'python3 {os.path.join(SCRIPTS, "teleop.py")} || read -p "teleop впав - Enter, щоб закрити"')
    if shutil.which('gnome-terminal'):
        term = ['gnome-terminal', '--title=Катер: керування', '--', 'bash', '-c', cmd]
    else:
        term = ['xterm', '-T', 'Катер: керування', '-e', 'bash', '-c', cmd]
    return ExecuteProcess(cmd=term, output='screen')


def launch_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    use_gt = LaunchConfiguration('use_gt').perform(context).lower() == 'true'
    nodes = [script('sensors_noise.py'), script('kf.py'),
             script('logger.py', f'tag:={mode}_gt' if use_gt else f'tag:={mode}')]
    if mode == 'manual':
        nodes.append(teleop_window())
    else:
        params = [f'mode:={mode}', f'use_gt:={use_gt}']
        for name in CONTROLLER_ARGS:
            value = LaunchConfiguration(name).perform(context)
            if value:
                # heading_kp:=5 ROS прочитав би як int, а параметр має тип double
                params.append(f'{name}:={value if name == "seed" else float(value)}')
        nodes.append(script('controller.py', *params))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('auto', default_value='true',
                              description='шум + EKF + автопілот + логер'),
        DeclareLaunchArgument('mode', default_value='square',
                              description='square circle figure8 zigzag fwd_back random steps manual'),
        DeclareLaunchArgument('use_gt', default_value='false',
                              description='регулятор по ground truth замість EKF'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(ROOT, 'models')),
        OpaqueFunction(function=launch_gz),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': BRIDGE_CONFIG, 'use_sim_time': True}],
            output='screen',
        ),
        *[DeclareLaunchArgument(name, default_value='', description='порожньо - як у controller.py')
          for name in CONTROLLER_ARGS],
        GroupAction(condition=IfCondition(LaunchConfiguration('auto')), actions=[
            OpaqueFunction(function=launch_nodes),
        ]),
    ])
