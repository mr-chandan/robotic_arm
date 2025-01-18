import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():
    # Package names
    package_name = 'robotic_arm'
    moveit_config_package = 'moveit_config_robotic_arm'

    # Get package paths
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    pkg_moveit_config = FindPackageShare(package=moveit_config_package).find(moveit_config_package)

    # Configuration file paths
    urdf_file_path = 'models/robot_arm.urdf.xacro'
    srdf_file_path = 'config/robot.srdf'
    controllers_file_path = 'config/moveit_controllers.yaml'
    joint_limits_file_path = 'config/joint_limits.yaml'
    kinematics_file_path = 'config/kinematics.yaml'
    rviz_config_file_path = 'config/move_group.rviz'

    # Set full paths
    urdf_path = os.path.join(pkg_share, urdf_file_path)
    srdf_path = os.path.join(pkg_moveit_config, srdf_file_path)
    controllers_path = os.path.join(pkg_moveit_config, controllers_file_path)
    joint_limits_path = os.path.join(pkg_moveit_config, joint_limits_file_path)
    kinematics_path = os.path.join(pkg_moveit_config, kinematics_file_path)
    rviz_config_path = os.path.join(pkg_moveit_config, rviz_config_file_path)

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2'
    )

    # Configure MoveIt
    moveit_config = (
        MoveItConfigsBuilder("robotic_arm", package_name=moveit_config_package)
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(file_path=controllers_path)
        .joint_limits(file_path=joint_limits_path)
        .robot_description_kinematics(file_path=kinematics_path)
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .to_moveit_configs()
    )

    # Start move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time}
        ],
    )

    # Start RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time}
        ],
        output="screen",
    )

    # Handle RViz shutdown
    rviz_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=EmitEvent(event=Shutdown())
        )
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)
    ld.add_action(rviz_exit_handler)

    return ld