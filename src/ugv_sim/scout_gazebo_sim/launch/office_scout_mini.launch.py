import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = "scout_description"
    gazebo_pkg      = "scout_gazebo_sim"
    nav_pkg         = "scout_navigation"

    description_share = FindPackageShare(description_pkg).find(description_pkg)
    gazebo_share      = FindPackageShare(gazebo_pkg).find(gazebo_pkg)
    nav_share         = FindPackageShare(nav_pkg).find(nav_pkg)

    urdf_xacro_path  = os.path.join(description_share, "urdf/mini.xacro")
    world_file_path  = os.path.join(gazebo_share, "worlds/office.world")
    sdf_model_path   = os.path.join(gazebo_share, "models/mini/model.sdf")  # 새로 추가된 SDF 파일 경로

    home = os.path.expanduser("~")
    cmd_vel_switch_path = os.path.join(home, "scout_ws/src/ugv_sim/scout_navigation/scripts/cmd_vel_switch.py")
    dock_script_path    = os.path.join(home, "scout_ws/src/ugv_sim/scout_navigation/scripts/waypoint_follower.py")

    controllers_yaml = os.path.join(gazebo_share, "config/controllers.yaml")

    map_yaml_default    = os.path.join(nav_share, "maps/office_world/office_world.yaml")
    nav2_params_default = os.path.join(nav_share, "params/office_world/nav2_params.yaml")
    rviz_config_default = os.path.join(nav_share, "rviz/office_nav2_config.rviz")
    ekf_params_default  = os.path.join(gazebo_share, "config/ekf.yaml")
    
    # ------------------ 추가된 기능 ------------------
    # SLAM, 네임스페이스, 헤드리스 모드, Rviz, 로봇 스폰 관련 변수 선언
    # 이 변수들은 위의 런치 파일에 있던 기능들을 구현하기 위해 추가됨
    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="false", description="Whether to run SLAM"
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace", default_value="false", description="Whether to apply a namespace to the navigation stack"
    )
    declare_headless_cmd = DeclareLaunchArgument(
        "headless", default_value="False", description="Whether to execute gzclient"
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )
    # --------------------------------------------------

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(gazebo_share, "models")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_gui          = DeclareLaunchArgument("gui", default_value="true")
    declare_spawn_x      = DeclareLaunchArgument("spawn_x", default_value="0.0")
    declare_spawn_y      = DeclareLaunchArgument("spawn_y", default_value="1.0")
    declare_spawn_z      = DeclareLaunchArgument("spawn_z", default_value="0.0")
    declare_spawn_yaw    = DeclareLaunchArgument("spawn_yaw", default_value="0.0")
    declare_map_yaml     = DeclareLaunchArgument("map_yaml", default_value=map_yaml_default)
    declare_params_file  = DeclareLaunchArgument("params_file", default_value=nav2_params_default)
    declare_autostart    = DeclareLaunchArgument("autostart", default_value="true")
    declare_rviz_config  = DeclareLaunchArgument("rviz_config", default_value=rviz_config_default)
    declare_use_dock     = DeclareLaunchArgument("use_dock", default_value="false")
    declare_ekf_params   = DeclareLaunchArgument("ekf_params", default_value=ekf_params_default)
    
    # ------------------ 추가된 기능 ------------------
    # 위에서 선언된 launch 인자들을 변수로 할당
    slam = LaunchConfiguration("slam")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    headless = LaunchConfiguration("headless")
    # --------------------------------------------------

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
    use_dock     = LaunchConfiguration("use_dock",  default_value="false")
    ekf_params   = LaunchConfiguration("ekf_params")

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("gazebo_ros").find("gazebo_ros"), "launch/gzserver.launch.py")
        ),
        launch_arguments={"world": world_file_path}.items(),
    )

    # ------------------ 추가된 기능 ------------------
    # 헤드리스 모드(GUI 없이 실행) 기능 추가
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("gazebo_ros").find("gazebo_ros"), "launch/gzclient.launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", gui, "' == 'true'"])),
    )
    # --------------------------------------------------

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
        condition=IfCondition(PythonExpression(["'", gui, "' == 'true'"])),
    )

    # ------------------ 추가된 기능 ------------------
    # 로봇 모델을 URDF 대신 SDF 파일로 스폰하는 기능 추가
    # SDF 파일을 사용하여 로봇을 스폰함 (위 코드와 동일)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "scout_mini",
            "-file",   sdf_model_path,
            "-x",      spawn_x,
            "-y",      spawn_y,
            "-z",      spawn_z,
            "-Y",      spawn_yaw,
        ],
        output="screen",
    )
    # --------------------------------------------------

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params, {"use_sim_time": use_sim_time}],
    )

    # ------------------ 추가된 기능 ------------------
    # SLAM, 네임스페이스 등 추가된 launch 인자들을 nav2_bringup에 전달
    # nav2_bringup 패키지의 launch/bringup_launch.py 파일을 참조함
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare("nav2_bringup").find("nav2_bringup"), "launch/bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map":          map_yaml,
            "params_file":  params_file,
            "autostart":    autostart,
            "slam":         slam,          # SLAM 기능 추가
            "namespace":    namespace,     # 네임스페이스 기능 추가
            "use_namespace": use_namespace, # 네임스페이스 기능 추가
        }.items(),
    )
    # --------------------------------------------------
    
    # ------------------ 추가된 기능 ------------------
    # use_rviz 인자에 따라 Rviz2 실행 여부 결정
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(["'", use_rviz, "' == 'true'"])),
    )
    # --------------------------------------------------

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

    dock_navigator = ExecuteProcess(
        cmd=["python3", dock_script_path, "--ros-args", "--log-level", "info"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", use_dock, "' == 'true'"])),
    )

    cmd_vel_switch = ExecuteProcess(
        cmd=[
            "python3", cmd_vel_switch_path,
            "--ros-args",
            "-p", "rate_hz:=50.0",
            "-p", "timeout_sec:=0.5",
            "-p", "joy_topic:=/cmd_vel_joy",
            "-p", "nav_topic:=/cmd_vel_nav",
            "-p", "out_topic:=/cmd_vel"
        ],
        output="screen",
    )

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen'
    )

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

    ld = LaunchDescription()

    # ------------------ 추가된 기능 ------------------
    # SLAM, 네임스페이스, 헤드리스 모드, Rviz 등 추가된 launch 인자들을 선언
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_headless_cmd)
    # --------------------------------------------------

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
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_dock)
    ld.add_action(declare_ekf_params)

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(robot_state_pub)
    ld.add_action(joint_state_gui)
    ld.add_action(TimerAction(period=3.0, actions=[spawn_entity]))
    ld.add_action(TimerAction(period=8.0, actions=[spawner_js]))
    ld.add_action(TimerAction(period=9.0, actions=[spawner_grip]))
    ld.add_action(TimerAction(period=9.7, actions=[spawner_height]))
    ld.add_action(TimerAction(period=10.5, actions=[ekf_node]))
    ld.add_action(TimerAction(period=11.0, actions=[nav2_bringup]))
    ld.add_action(TimerAction(period=14.0, actions=[rviz_node]))
    ld.add_action(dock_navigator)
    ld.add_action(cmd_vel_switch)
    ld.add_action(rosbridge)
    ld.add_action(web_video)

    return ld