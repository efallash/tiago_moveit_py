# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_description.launch_arguments import TiagoArgs
from tiago_description.tiago_launch_utils import get_tiago_hw_suffix

from dataclasses import dataclass
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


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

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


    srdf_file_path = Path(
        os.path.join(
            get_package_share_directory("tiago_moveit_config"),
            "config", "srdf",
            "tiago.srdf.xacro",
        )
    )

    kinematics_file_path = os.path.join(get_package_share_directory("tiago_moveit_config"), 'config', 'kinematics_kdl.yaml')
    pilz_limit_file_path = os.path.join(get_package_share_directory("tiago_moveit_config"), 'config', 'pilz_cartesian_limits.yaml')


    srdf_input_args = {
        "arm_type": arm_type,
        "end_effector": end_effector,
        "ft_sensor": ft_sensor,
        "base_type": base_type,
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (
        f'config/controllers/controllers{hw_suffix}.yaml')

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # The robot description is read from the topic /robot_description if the parameter is empty
    moveit_config = (
        MoveItConfigsBuilder('tiago')
        .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .pilz_cartesian_limits(file_path=pilz_limit_file_path)
        .moveit_cpp(
        file_path=get_package_share_directory("tiago_moveit_py")
        + "/config/motion_planning.yaml")
    )

    if use_sensor_manager:
        # moveit_sensors path
        moveit_sensors_path = 'config/sensors_3d.yaml'
        moveit_config.sensors_3d(moveit_sensors_path)

    moveit_config.to_moveit_configs()

    move_group_configuration = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "publish_robot_description_semantic": True,
        "robot_description_timeout": 90.0
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    moveit_py_node = Node(
        name="tiago_commander",
        package="tiago_moveit_py",
        executable="tiago_commander",
        output="both",
        parameters=move_group_params,
    )

    return [moveit_py_node]