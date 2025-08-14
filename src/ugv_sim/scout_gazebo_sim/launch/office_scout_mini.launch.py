import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # 패키지 경로
    description_pkg = "scout_description"
    gazebo_pkg      = "scout_gazebo_sim"
    nav_pkg         = "scout_navigation"

    description_share = FindPackageShare(description_pkg).find(description_pkg)
    gazebo_share      = FindPackageShare(gazebo_pkg).find(gazebo_pkg)
    nav_share         = FindPackageShare(nav_pkg).find(nav_pkg)

    urdf_xacro_path  = os.path.join(description_share, "urdf/mini.xacro")
    world_file_path  = os.path.join(gazebo_share, "worlds/office.world")
    dock_script_path = os.path.join(gazebo_share, "scripts/scout_navigate.py")
    controllers_yaml = os.path.join(gazebo_share, "config/controllers.yaml")  # gripper 컨트롤러 설정 파일

    map_yaml_default    = os.path.join(nav_share, "maps/office_world/office_world.yaml")
    nav2_params_default = os.path.join(nav_share, "params/office_world/nav2_params.yaml")
    rviz_config_default = os.path.join(nav_share, "rviz/office_nav2_config.rviz")
    # rviz_config_default = os.path.join(nav_share, "rviz/basic.rviz")

    # Gazebo 모델 경로 환경변수
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(gazebo_share, "models")

    # DeclareLaunchArgument
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock")

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Start Gazebo GUI client")

    declare_spawn_x = DeclareLaunchArgument(
        "spawn_x", default_value="0.0", description="Spawn X coordinate")

    declare_spawn_y = DeclareLaunchArgument(
        "spawn_y", default_value="1.0", description="Spawn Y coordinate")

    declare_spawn_z = DeclareLaunchArgument(
        "spawn_z", default_value="0.0", description="Spawn Z coordinate")

    declare_spawn_yaw = DeclareLaunchArgument(
        "spawn_yaw", default_value="0.0", description="Spawn yaw (rad)")

    declare_map_yaml = DeclareLaunchArgument(
        "map_yaml", default_value=map_yaml_default, description="Map YAML file")

    declare_params_file = DeclareLaunchArgument(
        "params_file", default_value=nav2_params_default, description="Nav2 parameters YAML")

    declare_autostart = DeclareLaunchArgument(
        "autostart", default_value="true", description="Autostart Nav2 stack")

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config", default_value=rviz_config_default, description="RViz config file")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz?")

    # LaunchConfiguration 객체
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui          = LaunchConfiguration("gui")
    spawn_x      = LaunchConfiguration("spawn_x")
    spawn_y      = LaunchConfiguration("spawn_y")
    spawn_z      = LaunchConfiguration("spawn_z")
    spawn_yaw    = LaunchConfiguration("spawn_yaw")
    map_yaml     = LaunchConfiguration("map_yaml")
    params_file  = LaunchConfiguration("params_file")
    autostart    = LaunchConfiguration("autostart")
    rviz_config  = LaunchConfiguration("rviz_config")
    use_rviz     = LaunchConfiguration("use_rviz")

    # Gazebo 서버·클라이언트
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("gazebo_ros").find("gazebo_ros"), "launch/gzserver.launch.py")
        ),
        launch_arguments={"world": world_file_path}.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("gazebo_ros").find("gazebo_ros"), "launch/gzclient.launch.py")
        ),
        condition=IfCondition(gui),
    )

    # 로봇 State / Joint GUI
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_xacro_path]),
            "use_sim_time": use_sim_time,
        }],
    )

    joint_state_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(gui),
    )

    # Spawn Entity (3 s delay)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "scout_mini",
            "-topic",  "robot_description",
            "-x",      spawn_x,
            "-y",      spawn_y,
            "-z",      spawn_z,
            "-Y",      spawn_yaw,
        ],
        output="screen",
    )

    # gripper와 joint state 브로드캐스터 컨트롤러 스포너
    entity_name = "scout_mini"
    manager_ns  = "/controller_manager"

    spawner_js = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", manager_ns],
        output="screen",
    )

    spawner_grip = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", manager_ns, "--param-file", controllers_yaml],
        output="screen",
    )

    spawner_height = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_height_controller", "--controller-manager", manager_ns, "--param-file", controllers_yaml],
        output="screen",
    )


    # Nav2 Bring-up
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("nav2_bringup").find("nav2_bringup"), "launch/bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map":          map_yaml,
            "params_file":  params_file,
            "autostart":    autostart,
        }.items(),
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    # 도킹 스크립트
    dock_navigator = ExecuteProcess(
        cmd=["python3", dock_script_path, "--ros-args", "--log-level", "info"],
        output="screen",
    )

    # rosbridge: 웹소켓 통신 (MapVisualizer 등 웹 클라이언트 연동용)
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen'
    )

    # web_video_server: 로봇 카메라 스트리밍
    web_video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{
            'ros_topic': '/depth_camera/image_raw',
            'image_transport': 'compressed'
        }]
    )

    # LaunchDescription 조립
    ld = LaunchDescription()

    # Declare arguments (개별 추가)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_gui)
    ld.add_action(declare_spawn_x)
    ld.add_action(declare_spawn_y)
    ld.add_action(declare_spawn_z)
    ld.add_action(declare_spawn_yaw)
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_params_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_rviz_config)
    ld.add_action(declare_use_rviz)

    # Core nodes & includes
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(robot_state_pub)
    ld.add_action(joint_state_gui)
    ld.add_action(TimerAction(period=3.0, actions=[spawn_entity]))
    ld.add_action(TimerAction(period=8.0, actions=[spawner_js]))
    ld.add_action(TimerAction(period=9.7, actions=[spawner_height]))
    ld.add_action(nav2_bringup)
    ld.add_action(rviz_node)
    ld.add_action(dock_navigator)
    ld.add_action(rosbridge)
    ld.add_action(web_video)

    return ld
