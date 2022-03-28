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
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)

    # Load URDF file from "xacro", so changes to them can be loaded without rebuilding the urdf file beforehand
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
    kinematics_yaml = load_yaml(
        'dual_panda_bringup', 'moveit_config/kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml}

    # Load Joint Limits
    joint_limits_yaml = load_yaml(
        'dual_panda_bringup', 'moveit_config/joint_limits.yaml')
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
            "resample_dt": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(
        "dual_panda_bringup", "moveit_config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    controllers_yaml = load_yaml(
        'dual_panda_bringup', 'moveit_config/controllers_dual_panda.yaml')
    print(controllers_yaml)
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml,
                          'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.execution_duration_monitoring': False,
                            'trajectory_execution.allowed_execution_duration_scaling': 100.0,
                            'trajectory_execution.allowed_goal_duration_margin': 100.0,
                            'trajectory_execution.allowed_start_tolerance': 0.01}

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'])

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[robot_description],
                                      output='both')

    # RViz
    pkg_share = FindPackageShare('dual_panda_bringup').find('dual_panda_bringup')
    rviz_config_file = os.path.join(pkg_share, 'config', 'dual_panda.rviz')
    rviz_node = Node(  # name='rviz2',
        package='rviz2',
        executable='rviz2',
        parameters=[robot_description,
                    robot_description_semantic,
                    ompl_planning_pipeline_config,
                    kinematics_yaml],
        output='screen',
        arguments=['-d', rviz_config_file])

    # Moveit move_group pkg
    move_group_interface = Node(package='moveit_ros_move_group',
                                executable='move_group',
                                output='screen',
                                parameters=[robot_description,
                                            robot_description_semantic,
                                            kinematics_yaml,
                                            joint_limits,
                                            ompl_planning_pipeline_config,
                                            trajectory_execution,
                                            moveit_controllers,
                                            planning_scene_monitor_parameters])

    # ros2_control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_panda_bringup"),
        "moveit_config",
        "ros_controllers_dual_panda.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner_left = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_left_controller",
                   "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner_right = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_right_controller",
                   "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([rviz_node, static_tf, robot_state_publisher_node, move_group_interface, ros2_control_node, joint_state_broadcaster_spawner, robot_controller_spawner_left, robot_controller_spawner_right])
