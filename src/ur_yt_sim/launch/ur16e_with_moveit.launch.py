import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    IncludeLaunchDescription,  # <-- add this
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # --- Launch configurations ---
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    # Paths
    controllers_yaml = PathJoinSubstitution([FindPackageShare(runtime_config_package), "config", controllers_file])
    initial_positions_yaml = PathJoinSubstitution([FindPackageShare(runtime_config_package), "config", initial_positions_file])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "rviz", "view_ur16e.rviz"])
    world_file = os.path.join(get_package_share_directory("ur_yt_sim"), "worlds", "world4.world")
    
    # --- Robot Description ---
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
        "safety_limits:=", safety_limits, " ",
        "safety_pos_margin:=", safety_pos_margin, " ",
        "safety_k_position:=", safety_k_position, " ",
        "name:=", ur_type, " ",
        "ur_type:=", ur_type, " ",
        "prefix:=", prefix, " ",
        "sim_gazebo:=true", " ",
        "simulation_controllers:=", controllers_yaml, " ",
        "initial_positions_file:=", initial_positions_yaml
    ])
    robot_description = {"robot_description": robot_description_content}

    # --- Nodes ---
    # robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # ros2_control_node (required for controllers)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
    ),
    launch_arguments={
        "world": world_file,
        "gui": gazebo_gui,
    }.items(),
    )

    # Spawn robot in Gazebo after ros2_control_node is running
    spawn_ur = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ros2_control_node,
            on_start=[Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", "ur", "-topic", "robot_description"],
                output="screen",
            )]
        )
    )

    # Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
        output="screen",
    )

    # MoveIt launch (delayed until robot_state_publisher is ready)
    ur_moveit_pkg = get_package_share_directory('ur_moveit_config')
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_pkg, 'launch', 'ur_moveit.launch.py')),
        launch_arguments={
            'ur_type': ur_type.perform(context),
            'use_rviz': 'false',
            'use_sim_time': 'true',
            'load_robot_description': 'true',
            'spawn_robot': 'false',
        }.items()
    )
    moveit_launch_delayed = TimerAction(
        period=5.0,
        actions=[moveit_launch]
    )

    # RViz (delayed until joint_state_broadcaster spawner finishes)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    rviz_delayed = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[rviz_node]
        )
    )

    return [
        gazebo_launch,
        robot_state_publisher_node,
        ros2_control_node,
        spawn_ur,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        moveit_launch_delayed,
        rviz_delayed
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument(
        "ur_type", default_value="ur16e", description="Type of UR robot."
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "safety_limits", default_value="true", description="Enable safety limits"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "safety_pos_margin", default_value="0.15", description="Safety margin"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "safety_k_position", default_value="20", description="Safety K position"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "runtime_config_package", default_value="ur_yt_sim", description="Controller config package"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "controllers_file", default_value="ur16e_controllers.yaml", description="Controllers YAML"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "initial_positions_file", default_value="initial_positions.yaml", description="Initial joint positions"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "description_package", default_value="ur_yt_sim", description="Package containing URDF/XACRO"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "description_file", default_value="ur16e_camera.urdf.xacro", description="URDF/XACRO file"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "prefix", default_value='""', description="Joint name prefix for multi-robot setup"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "start_joint_controller", default_value="true", description="Start joint trajectory controller"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "initial_joint_controller", default_value="joint_trajectory_controller", description="Initial joint controller"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "launch_rviz", default_value="true", description="Launch RViz?"
    ))
    declared_arguments.append(DeclareLaunchArgument(
        "gazebo_gui", default_value="true", description="Launch Gazebo GUI?"
    ))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
