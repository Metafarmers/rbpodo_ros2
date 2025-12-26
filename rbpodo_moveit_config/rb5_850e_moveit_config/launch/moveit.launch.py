import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from mflib.common.behavior_tree_impl_v3 import BehaviorTreeServerNodeV3


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description="Robot IP address",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):
    use_sim_time = BehaviorTreeServerNodeV3.get_use_sim_time()
    # Get launch arguments, with fallback to defaults
    robot_ip_arg = context.launch_configurations.get('robot_ip', '')
    use_fake_hardware_arg = context.launch_configurations.get('use_fake_hardware', '')

    # Use launch arguments if provided, otherwise use defaults
    if robot_ip_arg:
        robot_ip = robot_ip_arg
    else:
        robot_ip = "10.0.2.7" if use_sim_time else "192.168.50.101"

    if use_fake_hardware_arg:
        use_fake_hardware = use_fake_hardware_arg
    else:
        use_fake_hardware = "true" if use_sim_time else "false"

    use_rviz = context.launch_configurations.get('use_rviz', 'true')

    mappings = {
        "robot_ip": robot_ip,
        "cb_simulation": use_fake_hardware,
        "use_fake_hardware": use_fake_hardware,
        "fake_sensor_commands": 'false', #use_fake_hardware,
    }

    moveit_config = (
        MoveItConfigsBuilder("rb5_850e")
        .robot_description(file_path="config/rb5_850e.urdf.xacro", mappings=mappings)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": use_sim_time}],
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rb5_850e_moveit_config"), "config", rviz_base]
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time}
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        # parameters=[moveit_config.robot_description,
        #             ],
        parameters=[
            dict(moveit_config.robot_description),
            {"use_sim_time": use_sim_time},
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("rbpodo_bringup"),
        "config",
        "controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": use_sim_time},   # 추가
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],   # 추가
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],   # 추가
    )

    nodes_to_start = [
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]

    # Only add rviz_node if use_rviz is true
    if use_rviz.lower() == 'true':
        nodes_to_start.append(rviz_node)

    return nodes_to_start
