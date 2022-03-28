import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # planning_context
    robot_description_config = xacro.process_file(os.path.join(
        get_package_share_directory("dual_panda_bringup"),
        "urdf",
        "dual_panda.xacro")
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Load SRDF file from "support" package
    srdf_file = load_file('dual_panda_bringup', 'urdf/dual_panda.srdf')
    robot_description_semantic = {'robot_description_semantic': srdf_file}

    # Load Kinematics
    kinematics_yaml = load_yaml('dual_panda_bringup', 'moveit_config/kinematics.yaml')

    # MoveGroupInterface demo executable
    run_move_group_demo = Node(
        package="dual_panda_bringup",
        executable="planning_test",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml
        ],
    )

    return LaunchDescription([run_move_group_demo])