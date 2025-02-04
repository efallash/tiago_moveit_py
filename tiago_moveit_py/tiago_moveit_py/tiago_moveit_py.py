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
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters, 
#    TrajectoryExecutionManager
)


from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf_transformations import euler_from_quaternion

class TiagoPy():
    def __init__(self, name='moveit_py'):

        self.logger = get_logger(f'tiago.{name}')
        # instantiate MoveItPy instance and get planning component
        self.thor = MoveItPy(node_name=name)
        #self.execution_manager=self.thor.get_trajectory_execution_manager()
        self.thor_arm = self.thor.get_planning_component("arm")
        self.logger.info("MoveItPy instance created")

    def arm_go_to_named_pose(self, pose_name, sleep_time=0): #TODO: Add sleep time to all methods
        self.thor_arm.set_start_state_to_current_state()
        self.thor_arm.set_goal_state(configuration_name=pose_name)
        self.logger.info(f"Moving to pose: {pose_name}")
        return self.plan_and_execute(self.thor,self.thor_arm,self.logger, sleep_time=sleep_time)

    def arm_go_to_pose(self, pose: PoseStamped, sleep_time=0):
        self.thor_arm.set_start_state_to_current_state()
        self.thor_arm.set_goal_state(pose_stamped_msg=pose, pose_link="arm_tool_link")
        angles=euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        self.logger.info(f'Moving to pose x: {pose.pose.position.x} y: {pose.pose.position.y} z: {pose.pose.position.z} r: {angles[0]} p: {angles[1]} y: {angles[2]}')
        return self.plan_and_execute(self.thor,self.thor_arm,self.logger, sleep_time=sleep_time)

    def arm_go_to_random_pose(self, sleep_time=0):
        self.thor_arm.set_start_state_to_current_state()
        # instantiate a RobotState instance using the current robot model
        robot_model = self.thor.get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.set_to_random_positions()
        self.thor_arm.set_goal_state(robot_state=robot_state)
        self.logger.info("Moving to random joint positions")
        return self.plan_and_execute(self.thor,self.thor_arm,self.logger, sleep_time=sleep_time)

    def plan(self,
        planning_component,
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
            plan_result = planning_component.plan() #TODO: Add moveit.planning.PlanRequestParameters to control 
        return plan_result

    def execute(self,
        robot: MoveItPy,
        logger,
        plan_result,
        sleep_time=0.0        
        ):
        # execute the plan
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        result=robot.execute(robot_trajectory, controllers=[])
        time.sleep(sleep_time)
        return result
    
    def plan_and_execute(self,
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        
        logger.info('Planning and executing')
        plan_result=self.plan(planning_component,logger,single_plan_parameters,multi_plan_parameters)

        if plan_result:
            execute_result=self.execute(robot,logger,plan_result,sleep_time)
        else:
            logger.error('Planning failed')
            return (False, 'PLAN_FAILED')
        
        if execute_result:
            
            if execute_result.status=='SUCCEEDED':
                logger.info(f'EXECUTION SUCCEEDED')
                return (True, '')
            
            elif execute_result.status=='RUNNING':
                logger.info(f'EXECUTING')
                time.sleep(2)
                return (False, 'EXCEED_EXEC_TIME')
            else:

                logger.error(f'EXECUTION FAILED, code: {execute_result.status}')
                return (False, execute_result.status)
    def shutdown(self):
        self.thor.shutdown()


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    node=Node('tiago_test_node')
    tiago_moveit_py=TiagoPy()
    logger=node.get_logger()
    spin_thread=threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    ###########################################################################
    # Plan 2 - set goal state with RobotState object
    ###########################################################################

    logger.info('Moving to random pose')
    tiago_moveit_py.arm_go_to_random_pose()

    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    # set pose goal with PoseStamped message
    from tf_transformations import quaternion_from_euler
    from math import pi


    pose_goal = PoseStamped()
    orient = quaternion_from_euler(pi, 0, 0)
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.x = 0.420232
    pose_goal.pose.orientation.y = 0.534186
    pose_goal.pose.orientation.z = -0.439245
    pose_goal.pose.orientation.w = 0.587465
    pose_goal.pose.position.x = 0.648343
    pose_goal.pose.position.y = -0.185541
    pose_goal.pose.position.z = 0.769809

    # call thor method
    logger.info('Moving to goal pose and openning gripper')
    tiago_moveit_py.arm_go_to_pose(pose_goal)



if __name__ == "__main__":
    main()
