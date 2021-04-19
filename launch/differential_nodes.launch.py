import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    # Get the launch directory
    package_name = "differential_drive"
    package_dir = get_package_share_directory(package_name)

    # Launch configuration variables for parameters
    parameters_config_file = LaunchConfiguration('parameters_config_file')

    declare_parameters_config_file_cmd = DeclareLaunchArgument(
        'parameters_config_file',
        default_value=os.path.join(package_dir, 'config', 'T800.yaml'),
        description='Full path to the parameters config file to use')

    kinematics_node = Node(
        package=package_name,
        executable="kinematics",
        name="kinematics",
        parameters=[parameters_config_file])

    odometry_node = Node(
        package=package_name,
        executable="odometry_encoders",
        name="odometry_encoders",
        parameters=[parameters_config_file])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_parameters_config_file_cmd)

    ld.add_action(odometry_node)
    ld.add_action(kinematics_node)

    return ld
