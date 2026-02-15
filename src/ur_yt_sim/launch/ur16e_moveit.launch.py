import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    pkg_path = get_package_share_directory(package_name)
    full_path = os.path.join(pkg_path, file_path)
    with open(full_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    world_path = LaunchConfiguration("world")

    # --------------------------------------------------------
    #  ROBOT DESCRIPTION (URDF + ros2_control)
    # --------------------------------------------------------
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " ",
        "generate_ros2_control_tag:=true",
        " sim_ignition:=true",
        " sim_gazebo:=false",
        " use_fake_hardware:=false",
        " simulation_controllers:=",
        PathJoinSubstitution([
            FindPackageShare("ur_yt_sim"),
            "config",
            "ur16e_controllers.yaml"
        ])
    ])

    robot_description = {"robot_description": robot_description_content}

    # --------------------------------------------------------
    #  SRDF + MoveIt Config
    # --------------------------------------------------------
    srdf_path = os.path.join(
        get_package_share_directory("ur16e_moveit_config"),
        "config",
        "ur16e.srdf"
    )
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    kinematics_yaml = load_yaml("ur16e_moveit_config", "config/kinematics.yaml")
    planning_yaml = load_yaml("ur16e_moveit_config", "config/planning_pipelines.yaml")
    ompl_yaml = load_yaml("ur16e_moveit_config", "config/ompl_planning.yaml")
    moveit_controllers = load_yaml("ur16e_moveit_config", "config/moveit_controllers.yaml")
    robot_description_kinematics = {
        "robot_description_kinematics": kinematics_yaml
    }
    # --------------------------------------------------------
    #  IGNITION GAZEBO
    # --------------------------------------------------------
   
    ign_gazebo = ExecuteProcess(
    cmd=["ign", "gazebo", "-r", "-v", "4", world_path],
    output="screen"
    )


    # # --------------------------------------------------------
    # #  ROBOT STATE PUBLISHER
    # # --------------------------------------------------------
    robot_state_pub = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description, {"use_sim_time": True}],
                output="screen"
            )
        ]
    )


    # --------------------------------------------------------
    #  SPAWN ROBOT IN IGNITION
    # --------------------------------------------------------
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "ur",
            "-allow_renaming", "false"
        ]
    )
   
    bridge_params = PathJoinSubstitution([
        FindPackageShare("ur_yt_sim"),
        "config",
        "ign_bridge.yaml",
    ])

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_params}],
        output="screen",
    )

    # --------------------------------------------------------
    #  MOVEIT move_group (delayed until Gazebo loads)
    # --------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_yaml,
            {"ompl": ompl_yaml},
            moveit_controllers,
            {"use_sim_time": True},
            {"default_planning_pipeline": "ompl"},
        ],
    )

    delayed_move_group = TimerAction(period=12.0, actions=[move_group_node])

    # --------------------------------------------------------
    #  RVIZ2
    # --------------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        planning_yaml,
        {"ompl": ompl_yaml},
        {"use_sim_time": True},
    ],
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("ur16e_moveit_config"),
                "config",
                "moveit.rviz"
            ])
        ]
    )

    # --------------------------------------------------------
    #  CONTROLLERS (spawn in ROS2)
    # --------------------------------------------------------
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen"
    )


    return LaunchDescription([
        DeclareLaunchArgument("description_package", default_value="ur_yt_sim"),
        DeclareLaunchArgument("description_file", default_value="ur16e_with_evcharger.urdf.xacro"),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_yt_sim"),
                "worlds",
                "World_3.sdf"
            ])
        ),
    # 1. Start Gazebo immediately
    ign_gazebo,

    # 2. Spawn robot AFTER Gazebo is ready
    TimerAction(
        period=5.0,
        actions=[spawn_robot],
    ),

    # 3. Camera bridge
    TimerAction(
        period=7.0,
        actions=[gz_bridge],
    ),


    # 4. Spawn controllers AFTER robot is spawned
    TimerAction(
        period=8.0,
        actions=[joint_state_broadcaster],
    ),
    TimerAction(
        period=9.0,
        actions=[joint_trajectory_controller],
    ),

    # 5. Robot State Publisher immediately
    robot_state_pub,

    # 6. RViz after controllers are published
    TimerAction(
        period=10.0,
        actions=[rviz_node],
    ),

    # 7. MoveIt after everything is ready (12 seconds delay)
    delayed_move_group,

  
    # # 8. MoveIt Controller Node - Plan to static pose        
    # Node(
    #     package="ur_yt_sim",
    #     executable="moveit_controller",
    #     output="screen",
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         robot_description_kinematics,
    #         planning_yaml,
    #         {"ompl": ompl_yaml},
    #         moveit_controllers,
    #         {"use_sim_time": True},


    #     ],
    # )

     # 9. Preinsert Controller Node- plan to preinsert pose      
    Node(
        package="ur_yt_sim",
        executable="preinsert_controller",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_yaml,
            {"ompl": ompl_yaml},
            moveit_controllers,
            {"use_sim_time": True},
    

        ],
    ),
    #    Node(
    #     package="ur_yt_sim",
    #     executable="inlet_moveit_controller",
    #     output="screen",
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         robot_description_kinematics,
    #         planning_yaml,
    #         {"ompl": ompl_yaml},
    #         moveit_controllers,
    #         {"use_sim_time": True},
    

    #     ],
    # ),

],
)