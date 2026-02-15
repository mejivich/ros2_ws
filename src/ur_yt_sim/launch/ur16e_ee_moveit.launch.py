import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    full_path = os.path.join(pkg_path, file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    # Launch arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_path = LaunchConfiguration("world")

    # URDF (xacro → robot_description)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file])
    ])
    robot_description = {"robot_description": robot_description_content}

    # Load MoveIt configurations
    srdf_path = os.path.join(
        get_package_share_directory("ur16e_moveit_config"),
        "config",
        "ur16e.srdf"
    )
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    kinematics_yaml = load_yaml("ur16e_moveit_config", "config/kinematics.yaml")
    planning_yaml = load_yaml("ur16e_moveit_config", "config/planning_pipelines.yaml")
    ompl_planning_yaml = load_yaml("ur16e_moveit_config", "config/ompl_planning.yaml")
    controllers_yaml = load_yaml("ur16e_moveit_config", "config/moveit_controllers.yaml")

    # Gazebo simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            "gui": gazebo_gui,
            "world": world_path
        }.items(),
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen"
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "ur", "-topic", "robot_description"],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},

            # ✅ Load main planning pipeline config (contains OMPL/CHOMP/LERP definitions)
            planning_yaml,

            # ✅ Explicitly load ompl planner configs under their expected namespace
            {"ompl": ompl_planning_yaml},

            # ✅ Force OMPL as the default pipeline
            {"default_planning_pipeline": "ompl"},

            # ✅ Controller configuration
            controllers_yaml,

            # ✅ Time sync with Gazebo
            {"use_sim_time": True},
        ],
    )


    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("ur16e_moveit_config"), "config", "moveit.rviz"
            ])
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},

            # ✅ Load planning pipeline config directly
            planning_yaml,

            # ✅ Correct OMPL namespace
            {"ompl": ompl_planning_yaml},

            # ✅ Force OMPL to be default also inside RViz
            {"default_planning_pipeline": "ompl"},

            {"use_sim_time": True},
        ]
    )
    
    # Controllers
    joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    joint_trajectory_controller = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument("description_package", default_value="ur_yt_sim"),
        DeclareLaunchArgument("description_file", default_value="ur16e_with_evcharger.urdf.xacro"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),

        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_yt_sim"),
                "worlds",
                "ev_charging_world.world"
            ])
        ),

        gazebo,
        robot_state_pub,
        spawn_robot,
        move_group_node,
        rviz_node,
        joint_state_broadcaster,
        joint_trajectory_controller,
    ])
