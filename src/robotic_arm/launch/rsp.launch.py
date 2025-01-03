import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    namePackage = 'robotic_arm'
    modelFileRelativePath = 'models/robot_arm.urdf.xacro'

    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    robotDescription = xacro.process_file(pathModelFile).toxml()

 
    nodeRobotStatePublisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription , 'use_sim_time': True}]
    )

  
    LaunchDescriptionObject = LaunchDescription()

    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)


    return LaunchDescriptionObject




