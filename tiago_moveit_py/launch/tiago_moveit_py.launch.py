"""
A launch file for running the motion planning python api tutorial
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node
from tiago_description.tiago_launch_utils import get_tiago_hw_suffix
from moveit_configs_utils import MoveItConfigsBuilder
from launch_pal.arg_utils import LaunchArgumentsBase
from tiago_description.launch_arguments import TiagoArgs
from launch.substitutions import LaunchConfiguration
from dataclasses import dataclass
from launch_pal.robot_arguments import CommonArgs
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    base_type: DeclareLaunchArgument = TiagoArgs.base_type
    arm_type: DeclareLaunchArgument = TiagoArgs.arm_type
    end_effector: DeclareLaunchArgument = TiagoArgs.end_effector
    ft_sensor: DeclareLaunchArgument = TiagoArgs.ft_sensor
    wrist_model: DeclareLaunchArgument = TiagoArgs.wrist_model
    camera_model: DeclareLaunchArgument = TiagoArgs.camera_model
    laser_model: DeclareLaunchArgument = TiagoArgs.laser_model

    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    use_sensor_manager: DeclareLaunchArgument = CommonArgs.use_sensor_manager


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):

    launch_description.add_action(OpaqueFunction(function=start_move_group))
    return


def start_move_group(context, *args, **kwargs):

    base_type = read_launch_argument("base_type", context),
    arm_type = read_launch_argument("arm_type", context)
    end_effector = read_launch_argument("end_effector", context)
    ft_sensor = read_launch_argument("ft_sensor", context)
    use_sensor_manager = read_launch_argument("use_sensor_manager", context)

    hw_suffix = get_tiago_hw_suffix(
        arm=arm_type,
        end_effector=end_effector,
    )

    srdf_file_path = os.path.join(
        get_package_share_directory("tiago_moveit_config"),
        "config", "srdf",
        "tiago.srdf.xacro",
    )

    srdf_input_args = {
        "arm_type": arm_type,
        "end_effector": end_effector,
        "ft_sensor": ft_sensor,
        "base_type": base_type,
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = f"config/controllers/controllers{hw_suffix}.yaml"

    robot_description_kinematics = "config/kinematics_kdl.yaml"
    joint_limits = "config/joint_limits.yaml"
    pilz_cartesian_limits = "config/pilz_cartesian_limits.yaml"

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": False,
        "publish_transforms_updates": False,
        "publish_robot_description": False,
    }

    # The robot description is read from the topic /robot_description if the parameter is empty
    moveit_config = (
        MoveItConfigsBuilder("tiago")
        .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        .robot_description_kinematics(file_path=robot_description_kinematics)
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .joint_limits(file_path=joint_limits)
        .planning_pipelines(
            pipelines=["ompl"], default_planning_pipeline="ompl"
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits)
        .moveit_cpp(
            file_path=get_package_share_directory("tiago_moveit_py")
            + "/config/motion_planning.yaml"
        )
    )

    if use_sensor_manager:
        # moveit_sensors path
        moveit_sensors_path = "config/sensors_3d.yaml"
        moveit_config.sensors_3d(moveit_sensors_path)

    moveit_config.to_moveit_configs()

    move_group_configuration = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "publish_robot_description_semantic": True,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    moveit_py_node = Node(
        name="tiago_moveit_py_test",
        package="tiago_moveit_py",
        executable="tiago_moveit_py",
        output="both",
        parameters=move_group_params,
    )

    return [moveit_py_node]