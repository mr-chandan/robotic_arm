import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Package and model file
    namePackage = 'robotic_arm'  # Change this to your package name
    modelFileRelativePath = 'urdf/robot_arm_urdf.urdf'  # Path to your URDF file

    # Get the full path of the URDF file
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    # Read the URDF file as a string
    with open(pathModelFile, 'r') as file:
        robotDescription = file.read()

    # Node to publish the robot description
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # Return LaunchDescription
    return LaunchDescription([nodeRobotStatePublisher])