import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():

    namePackage = 'robotic_arm'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(namePackage),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    controller_params_file = os.path.join(get_package_share_directory(namePackage), 'config', 'robotic_arm_controllers.yaml')
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                   controller_params_file],
        name='controller_manager'
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )
    
    delayed_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[arm_controller_spawner] 
        )
    )
    
    LaunchDescriptionObject = LaunchDescription()

    LaunchDescriptionObject.add_action(rsp)
    LaunchDescriptionObject.add_action(delayed_controller_manager)
    LaunchDescriptionObject.add_action(delayed_joint_state_broadcaster_spawner)
    LaunchDescriptionObject.add_action(delayed_arm_controller_spawner)

    return LaunchDescriptionObject