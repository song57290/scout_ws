import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():

    # Package names
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')
    pkg_scout_gazebo = FindPackageShare('scout_gazebo_sim').find('scout_gazebo_sim')
    pkg_scout_description = FindPackageShare('scout_description').find('scout_description')

    # File paths
    urdf_file = os.path.join(pkg_scout_description, 'urdf', 'mini.xacro')
    rviz_config_file = os.path.join(pkg_scout_description, 'rviz', 'urdf.rviz')
    world_file = os.path.join(pkg_scout_gazebo, 'worlds', 'hospital.world')
    gazebo_models_path = os.path.join(pkg_scout_gazebo, 'models')

    # Set GAZEBO_MODEL_PATH
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_path

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    use_simulator = LaunchConfiguration('use_simulator')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_model = LaunchConfiguration('urdf_model', default=urdf_file)
    rviz_file = LaunchConfiguration('rviz_config_file', default=rviz_config_file)

    # Robot spawn pose
    spawn_x = '0.0'
    spawn_y = '0.0'
    spawn_z = '0.0'
    spawn_yaw = '0.0'

    # Declare launch arguments
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        DeclareLaunchArgument('gui', default_value='True', description='Use joint_state_publisher_gui'),
        DeclareLaunchArgument('headless', default_value='False', description='Run without GUI'),
        DeclareLaunchArgument('use_simulator', default_value='True', description='Launch Gazebo'),
        DeclareLaunchArgument('use_rviz', default_value='True', description='Launch RViz'),
        DeclareLaunchArgument('urdf_model', default_value=urdf_file, description='Path to URDF'),
        DeclareLaunchArgument('rviz_config_file', default_value=rviz_config_file, description='Path to RViz config'),
    ]

    # Nodes and launch files
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_model]), value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world_file}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'scout_mini',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
        ],
        output='screen'
    )

    # Launch description
    ld = LaunchDescription()

    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(spawn_entity)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz_node)

    return ld
