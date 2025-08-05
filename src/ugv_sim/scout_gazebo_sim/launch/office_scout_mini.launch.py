import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ───────────────── 패키지·경로 ─────────────────
    descr_pkg  = 'scout_description'
    gazebo_pkg = 'scout_gazebo_sim'

    descr_path  = FindPackageShare(descr_pkg).find(descr_pkg)
    gazebo_path = FindPackageShare(gazebo_pkg).find(gazebo_pkg)

    urdf_xacro  = os.path.join(descr_path, 'urdf/mini.xacro')
    world_file  = os.path.join(gazebo_path, 'worlds/office.world')
    dock_script = os.path.join(gazebo_path, 'scripts', 'navigate_to_charging_dock_v4.py')

    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(gazebo_path, 'models')

    # ───────────────── Launch 인자 ─────────────────
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui          = LaunchConfiguration('gui')
    spawn_x      = LaunchConfiguration('spawn_x')
    spawn_y      = LaunchConfiguration('spawn_y')
    spawn_z      = LaunchConfiguration('spawn_z')
    spawn_yaw    = LaunchConfiguration('spawn_yaw')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui',          default_value='true'),
        DeclareLaunchArgument('spawn_x',      default_value='0.0'),
        DeclareLaunchArgument('spawn_y',      default_value='1.0'),
        DeclareLaunchArgument('spawn_z',      default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw',    default_value='0.0'),
    ]

    # ───────────────── Gazebo ─────────────────
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('gazebo_ros').find('gazebo_ros'),
                         'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items())

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('gazebo_ros').find('gazebo_ros'),
                         'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui))

    # ───────────────── 로봇 TF & GUI ─────────────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_xacro]),
            'use_sim_time': use_sim_time
        }])

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(gui))

    # ───────────────── 로봇 스폰 (3 s 딜레이) ─────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'scout_mini',
            '-topic',  'robot_description',
            '-x',      spawn_x,
            '-y',      spawn_y,
            '-z',      spawn_z,
            '-Y',      spawn_yaw],
        output='screen')

    # ───────────────── 도킹 스크립트 실행 ─────────────────
    dock_navigator = ExecuteProcess(
        cmd=['python3', dock_script,
             '--ros-args', '--log-level', 'info'],
        output='screen')

    # ───────────────── LaunchDescription 조립 ─────────────────
    ld = LaunchDescription()

    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(rsp_node)
    ld.add_action(jsp_gui)
    ld.add_action(TimerAction(period=3.0, actions=[spawn_entity]))
    ld.add_action(dock_navigator)

    return ld
