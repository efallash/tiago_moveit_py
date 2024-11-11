#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time
import threading

# generic ros libraries
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.action import ActionClient

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.core.robot_model import RobotModel
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.planning import (
    MoveItPy,
    PlanningComponent,
    TrajectoryExecutionManager
)


from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf_transformations import euler_from_quaternion

class TiagoDualPy():
    def __init__(self, name='moveit_py'):

        self.logger = get_logger(f'tiago_dual.{name}')
        # instantiate MoveItPy instance and get planning component
        self.tiago_dual = MoveItPy(node_name=name)
        trajectory_execution = self.tiago_dual.get_trajactory_execution_manager()
        assert isinstance(trajectory_execution, TrajectoryExecutionManager)
        trajectory_execution.enable_execution_duration_monitoring(True)
        trajectory_execution.set_allowed_execution_duration_scaling(1.2)
        trajectory_execution.set_allowed_start_tolerance(0.05)

        # Create objects for the arms and grippers
        self.groups = {}
        self.gripper_links = {}
        self.grippers = {}
        self.gripper_goal_msgs = {}

        # Populate arms and grippers #TODO: Add support for controlling both arms in a single group
        self.groups['right'] = self.tiago_dual.get_planning_component("arm_right")
        self.groups['left'] = self.tiago_dual.get_planning_component("arm_left")
        self.groups['right_torso'] = self.tiago_dual.get_planning_component("arm_right_torso")
        self.groups['left_torso'] = self.tiago_dual.get_planning_component("arm_left_torso")
        self.groups['both_arms_torso'] = self.tiago_dual.get_planning_component("both_arms_torso")

        self.gripper_links['right'] = 'arm_right_tool_link'
        self.gripper_links['left'] = 'arm_left_tool_link'
        #ADD GRIPPER GROUPS
        self.logger.info("MoveItPy instance created")

    def arm_go_to_named_pose(self, arm: str, pose_name: str, vel_factor=0.2, sleep_time=0.1): #TODO: Add acceleration and velocity scaling to all methods
        if arm=='left' or arm=='right':
            selected_arm=self.groups[arm]
        else:
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
            return False
        assert isinstance(selected_arm, PlanningComponent)
        selected_arm.set_start_state_to_current_state()
        selected_arm.set_goal_state(configuration_name=pose_name)
        self.logger.info(f"Moving to pose: {pose_name}")
        return self.plan_and_execute(self.tiago_dual, selected_arm, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)
    
    def dual_arm_go_to_pose(self, pose_right: PoseStamped, pose_left: PoseStamped, torso_constraint, vel_factor=0.2, sleep_time=0.1):
        both_arms_torso=self.groups["both_arms_torso"]
        robot_model = self.tiago_dual.get_robot_model()
        planning_monitor=self.tiago_dual.get_planning_scene_monitor()
        assert isinstance(both_arms_torso, PlanningComponent)

        
        with planning_monitor.read_write() as scene:
            current_state = scene.current_state
            assert isinstance(current_state, RobotState)
            robot_state = RobotState(robot_model) 
            current_state.update(True)
        
            #************************DEBUG*****************************************************

            self.logger.info(f"Right: {current_state.get_pose('arm_right_tool_link')}")
            self.logger.info(f"Left: {current_state.get_pose('arm_left_tool_link')}")
            self.logger.info(f"Torso: {current_state.get_joint_group_positions('torso')[0]}")
             
            robot_state.set_joint_group_positions("torso", [torso_constraint])
            robot_state.update(True)
            self.logger.info("Planning right arm")
            robot_state.set_from_ik("arm_right", pose_right.pose, self.gripper_links["right"], 1.0)
            self.logger.info("Planning left arm")
            robot_state.set_from_ik("arm_left", pose_left.pose, self.gripper_links["left"], 1.0)
            robot_state.update(True) 

            #************************DEBUG*****************************************************
            self.logger.info(f"Right: {robot_state.get_pose('arm_right_tool_link')}")
            self.logger.info(f"Left: {robot_state.get_pose('arm_left_tool_link')}")
            self.logger.info(f"Torso: {robot_state.get_joint_group_positions('torso')[0]}")
            
        
        both_arms_torso.set_start_state_to_current_state()
        both_arms_torso.set_goal_state(robot_state=robot_state)
        return self.plan_and_execute(self.tiago_dual, both_arms_torso, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)


        

    def arm_go_to_pose(self, arm: str, pose: PoseStamped, vel_factor=0.2, sleep_time=0.1):
        if arm=='left' or arm=='right':
            selected_arm=self.groups[arm]
            gripper_link=self.gripper_links[arm]
        else:
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
            return False
        assert isinstance(selected_arm, PlanningComponent)
        selected_arm.set_start_state_to_current_state()
        selected_arm.set_goal_state(pose_stamped_msg=pose, pose_link=gripper_link)
        angles=euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        self.logger.info(f'Moving to pose x: {pose.pose.position.x} y: {pose.pose.position.y} z: {pose.pose.position.z} r: {angles[0]} p: {angles[1]} y: {angles[2]}')
        return self.plan_and_execute(self.tiago_dual, selected_arm, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)


    def plan(self,
        planning_component: PlanningComponent,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None
        ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()
        return plan_result

    def execute(self,
        robot: MoveItPy,
        logger,
        plan_result,
        vel_factor=1,
        sleep_time=0.0        
        ):
        # execute the plan
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        assert isinstance(robot_trajectory, RobotTrajectory)
        robot_trajectory.apply_totg_time_parameterization(vel_factor, 1.0)
        result=robot.execute(robot_trajectory, controllers=[])
        time.sleep(sleep_time)
        return result

    def plan_and_execute(self,
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        vel_factor=1,
        sleep_time=0.0,
    ):

        logger.info('Planning and executing')
        plan_result=self.plan(planning_component,logger,single_plan_parameters,multi_plan_parameters)

        if plan_result:
            if vel_factor>0:
                execute_result=self.execute(robot,logger,plan_result, vel_factor, sleep_time)

                if execute_result.status=='SUCCEEDED':
                    logger.info(f'EXECUTION SUCCEEDED')
                    return (True, execute_result.status)
                else:
                    logger.error(f'EXECUTION FAILED, code: {execute_result.status}')
                    return (False, execute_result.status)
            else:
                logger.info('Planning Suceeded')
                return (True, 'PLAN_SUCEEDED')
        else:
            logger.error('Planning failed')
            return (False, 'PLAN_FAILED')
    def shutdown(self):
        self.tiago_dual.shutdown()


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    tiago_dual=TiagoDualPy()
    logger=tiago_dual.logger
    ###########################################################################
    # Plan 2 - set goal state with RobotState object
    ###########################################################################

    #TODO

    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    #Right arm pose
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_footprint"
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = 0.707
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.707
    pose_goal.pose.position.x = 0.4
    pose_goal.pose.position.y = -0.3
    pose_goal.pose.position.z = 0.4

    # call arm method
    logger.info('Moving to goal pose 1 right')
    tiago_dual.arm_go_to_pose('right', pose_goal, vel_factor=1)

    #Left arm pose
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_footprint"
    pose_goal.pose.orientation.x = 0.707
    pose_goal.pose.orientation.y = 0.0
    pose_goal.pose.orientation.z = -0.707
    pose_goal.pose.orientation.w = 0.0
    pose_goal.pose.position.x = 0.4
    pose_goal.pose.position.y = 0.3
    pose_goal.pose.position.z = 0.4



    # call arm method
    logger.info('Moving to goal pose 1 left')
    tiago_dual.arm_go_to_pose('left', pose_goal, vel_factor=1)


    #Right arm pose
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_footprint"
    pose_goal.pose.orientation.x = 0.707
    pose_goal.pose.orientation.y = 0.0
    pose_goal.pose.orientation.z = 0.707
    pose_goal.pose.orientation.w = 0.0
    pose_goal.pose.position.x = 0.4
    pose_goal.pose.position.y = -0.3
    pose_goal.pose.position.z = 1.2


    # call arm method
    logger.info('Moving to goal pose 2 right')
    tiago_dual.arm_go_to_pose('right', pose_goal, vel_factor=1)



    pose_goal.header.frame_id = "base_footprint"
    pose_goal.pose.orientation.x = 0.0
    pose_goal.pose.orientation.y = -0.707
    pose_goal.pose.orientation.z = 0.0
    pose_goal.pose.orientation.w = 0.707
    pose_goal.pose.position.x = 0.4
    pose_goal.pose.position.y = 0.3
    pose_goal.pose.position.z = 1.2

        # call arm method
    logger.info('Moving to goal pose 2 left')
    tiago_dual.arm_go_to_pose('left', pose_goal, vel_factor=1)


    ###########################################################################
    # Plan 4 - set goal state for both arms with PoseStamped message
    ###########################################################################

    logger.info("Performing dual arm movement")

    #Right arm pose
    pose_goal_r = PoseStamped()
    pose_goal_r.header.frame_id = "base_footprint"
    pose_goal_r.pose.orientation.x = 0.0
    pose_goal_r.pose.orientation.y = 0.707
    pose_goal_r.pose.orientation.z = 0.0
    pose_goal_r.pose.orientation.w = 0.707
    pose_goal_r.pose.position.x = 0.4
    pose_goal_r.pose.position.y = -0.3
    pose_goal_r.pose.position.z = 0.4

    #Left arm pose
    pose_goal_l = PoseStamped()
    pose_goal_l.header.frame_id = "base_footprint"
    pose_goal_l.pose.orientation.x = 0.707
    pose_goal_l.pose.orientation.y = 0.0
    pose_goal_l.pose.orientation.z = -0.707
    pose_goal_l.pose.orientation.w = 0.0
    pose_goal_l.pose.position.x = 0.4
    pose_goal_l.pose.position.y = 0.3
    pose_goal_l.pose.position.z = 0.4

    logger.info('Moving to goal pose 1 dual')
    tiago_dual.dual_arm_go_to_pose(pose_goal_r, pose_goal_l, 0.1, vel_factor=1.0)

    #Right arm pose
    pose_goal_r.header.frame_id = "base_footprint"
    pose_goal_r.pose.orientation.x = 0.707
    pose_goal_r.pose.orientation.y = 0.0
    pose_goal_r.pose.orientation.z = 0.707
    pose_goal_r.pose.orientation.w = 0.0
    pose_goal_r.pose.position.x = 0.4
    pose_goal_r.pose.position.y = -0.3
    pose_goal_r.pose.position.z = 1.2


    #Left arm pose
    pose_goal_l.header.frame_id = "base_footprint"
    pose_goal_l.pose.orientation.x = 0.0
    pose_goal_l.pose.orientation.y = -0.707
    pose_goal_l.pose.orientation.z = 0.0
    pose_goal_l.pose.orientation.w = 0.707
    pose_goal_l.pose.position.x = 0.4
    pose_goal_l.pose.position.y = 0.3
    pose_goal_l.pose.position.z = 1.2

    logger.info('Moving to goal pose 2 dual')
    tiago_dual.dual_arm_go_to_pose(pose_goal_r, pose_goal_l, 0.3, vel_factor=1.0)





if __name__ == "__main__":
    main()
