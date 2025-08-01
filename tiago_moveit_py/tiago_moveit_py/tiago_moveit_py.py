#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time
import threading
import math

# generic ros libraries
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# moveit python library
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from moveit.core.robot_state import RobotState, robotStateToRobotStateMsg
from moveit.core.robot_model import RobotModel
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.core.planning_interface import MotionPlanResponse
from moveit.planning import (
    MoveItPy,
    PlanningComponent,
    TrajectoryExecutionManager, 
    PlanningSceneMonitor
)


from geometry_msgs.msg import PoseStamped, TransformStamped
from control_msgs.action import FollowJointTrajectory
import rclpy.task
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from tf_transformations import euler_from_quaternion
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class TiagoPy(Node):
    def __init__(self, name='moveit_py'):
        super().__init__(name)
        self.logger = self.get_logger()
        #Create callback groups
        self.cb_client = MutuallyExclusiveCallbackGroup()
        self.cb_internal_client = MutuallyExclusiveCallbackGroup()
        self.cb_server = MutuallyExclusiveCallbackGroup()

        # instantiate MoveItPy instance and get planning component
        self.tiago = MoveItPy(node_name=name)
        trajectory_execution = self.tiago.get_trajectory_execution_manager()
        self.robot_model = self.tiago.get_robot_model()
        self.planning_monitor=self.tiago.get_planning_scene_monitor()
        assert isinstance(self.robot_model, RobotModel)
        assert isinstance(self.planning_monitor, PlanningSceneMonitor)
        assert isinstance(trajectory_execution, TrajectoryExecutionManager)
        trajectory_execution.enable_execution_duration_monitoring(True)
        trajectory_execution.set_allowed_execution_duration_scaling(1.2)
        trajectory_execution.set_allowed_start_tolerance(0.05)

        #Service clients for the cartesian path planning and execution
        self.cartesian_plan_client = self.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=self.cb_internal_client)

        # Create objects for the arms and grippers
        self.groups = {}

        # Populate arms and grippers
        self.groups['arm'] = self.tiago.get_planning_component("arm")
        self.groups['arm_torso'] = self.tiago.get_planning_component("arm_torso")
        self.gripper = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory", callback_group=self.cb_client)
        self.gripper_link = 'gripper_grasping_frame'

        self.logger.info("MoveItPy instance created")

    def publish_grasping_frame(self):
        # Create a static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        # Create a static transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_footprint"
        transform.child_frame_id = "gripper_grasping_frame"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        # Send the transformation
        self.tf_broadcaster.sendTransform(transform)


    def get_joint_states(self, topic):
        self.logger.info("Obtaining joint states...")
        future = rclpy.task.Future()

        def callback(msg):
            if not future.done():
                future.set_result(msg)

        subscription = self.create_subscription(
            JointTrajectoryControllerState,
            topic,
            callback,
            1
        )

        rclpy.spin_until_future_complete(self, future)

        self.destroy_subscription(subscription)

        result = future.result()
        assert isinstance(result, JointTrajectoryControllerState)
        joint_names = result.joint_names
        joint_positions = list(result.reference.positions)
        joint_states = dict(zip(joint_names, joint_positions))

        return joint_states

    def move_head(self, left_right, up_down, vel_factor = 0.1, sleep_time = 0.1):
        action_name = "/head_controller/follow_joint_trajectory"
        action_client = ActionClient(self, FollowJointTrajectory, action_name)

        point = JointTrajectoryPoint()
        point.positions = [left_right, up_down]
        joint_states = self.get_joint_states("/head_controller/controller_state")
        delta_left_right = abs(joint_states['head_1_joint'] - left_right)
        delta_up_down = abs(joint_states['head_2_joint'] - up_down)
        distance = math.sqrt(delta_left_right**2 + delta_up_down**2)
        duration = distance/vel_factor
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        msg.trajectory.points = [point]
        self.logger.info(f"Moving head to: {left_right},{up_down}")
        future = action_client.send_goal_async(msg)
        time.sleep(duration + sleep_time)
        return future

    def close_gripper(self, vel_factor = 0.1, sleep_time=0.1):
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        self.logger.info("Closing gripper")
        future = self.gripper_action(point, vel_factor, sleep_time)
        return future

    def open_gripper(self, vel_factor = 0.1, sleep_time=0.1):
        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        self.logger.info("Opening gripper")
        future = self.gripper_action(point, vel_factor, sleep_time)
        return future

    def move_gripper (self, left_finger, right_finger, vel_factor = 0.1, sleep_time = 0.1):
        point = JointTrajectoryPoint()
        point.positions = [right_finger, left_finger]
        self.logger.info(f"Moving gripper to: {right_finger},{left_finger}")
        future = self.gripper_action(point, vel_factor, sleep_time)
        return future
    
    def gripper_action(self, point:JointTrajectoryPoint, vel_factor, sleep_time):        
        action_client = self.gripper
        msg = FollowJointTrajectory.Goal()
        
        with self.planning_monitor.read_write() as scene:
            current_state = scene.current_state
            assert isinstance(current_state, RobotState)
            current_state.update(True)
            joint_positions=current_state.get_joint_group_positions(f"gripper")

        joint_goals = list(point.positions)
        distances = [abs(joint_positions[0]-joint_goals[0]),abs(joint_positions[1]-joint_goals[1])]
        duration = (max(distances)/vel_factor)*2
        point.time_from_start.sec = int(duration)
        
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.trajectory.joint_names = [f"gripper_right_finger_joint", f"gripper_left_finger_joint"]
        msg.trajectory.points = [point]
        self.logger.debug(f"DEBUG - Sending goal {point} with duration {duration}")
        future = action_client.send_goal_async(msg)
        time.sleep(duration + sleep_time)
        return future

    def arm_go_to_named_pose(self, pose_name: str, vel_factor=0.2, sleep_time=0.1): #TODO: Add acceleration and velocity scaling to all methods
        selected_arm=self.groups["arm"]
        assert isinstance(selected_arm, PlanningComponent)
        selected_arm.set_start_state_to_current_state()
        selected_arm.set_goal_state(configuration_name=pose_name)
        self.logger.info(f"Moving to pose: {pose_name}")
        return self.plan_and_execute(self.tiago, selected_arm, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)

    def arm_go_to_pose(self, pose: PoseStamped, vel_factor=0.2, sleep_time=0.1):
        selected_arm = self.groups["arm"]
        gripper_link=self.gripper_link
        assert isinstance(selected_arm, PlanningComponent)
        selected_arm.set_start_state_to_current_state()
        selected_arm.set_goal_state(pose_stamped_msg=pose, pose_link=gripper_link)
        angles=euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        self.logger.info(f'Moving to pose x: {pose.pose.position.x} y: {pose.pose.position.y} z: {pose.pose.position.z} r: {angles[0]} p: {angles[1]} y: {angles[2]}')
        return self.plan_and_execute(self.tiago, selected_arm, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)

    def arm_go_to_random_pose(self, sleep_time=0):
        selected_arm = self.groups["arm"]
        assert isinstance(selected_arm, PlanningComponent)
        selected_arm.set_start_state_to_current_state()
        # instantiate a RobotState instance using the current robot model
        robot_model = self.tiago.get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.set_to_random_positions()
        selected_arm.set_goal_state(robot_state=robot_state)
        self.logger.info("Moving to random joint positions")
        return self.plan_and_execute(self.tiago,selected_arm,self.logger, sleep_time=sleep_time)
        
    def torso_go_to_position(self, position: float, vel_factor=0.2, sleep_time=0.1):
        #TODO: No "torso" group in the moveit config
        raise NotImplementedError
        torso=self.groups["torso"]
        assert isinstance(torso, PlanningComponent)
        robot_state = RobotState(self.robot_model) 
        robot_state.set_joint_group_positions("torso", [position])
        robot_state.update(True)
        torso.set_goal_state(robot_state=robot_state)
        self.logger.info(f"Moving torso to position: {position} m")
        return self.plan_and_execute(self.tiago, torso, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)
    
    async def arm_go_to_pose_cartesian(self, pose: PoseStamped, vel_factor=0.2, sleep_time=0.1):
        self.logger.info('Planning and executing cartesian path')
        plan_result, robot_trajectory =await self.cartesian_plan(pose)
        if plan_result:
            if vel_factor>0:
                execute_result=self.execute(self.tiago, self.logger, trajectory=robot_trajectory, vel_factor=vel_factor, sleep_time=sleep_time)

                if execute_result.status=='SUCCEEDED':
                    self.logger.info(f'EXECUTION SUCCEEDED')
                    return (True, execute_result.status)
                else:
                    self.logger.error(f'EXECUTION FAILED, code: {execute_result.status}')
                    return (False, execute_result.status)
            else:
                self.logger.info('Planning Suceeded')
                return (True, 'PLAN_SUCEEDED')
        else:
            self.logger.error('Planning failed')
            return (False, 'PLAN_FAILED')

    async def cartesian_plan(self, arm:str, pose:PoseStamped):
        selected_arm = self.groups["arm"]
        gripper_link=self.gripper_link
        #Define request message
        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = pose.header.frame_id
        request.link_name = gripper_link
        request.group_name = selected_arm
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        #Obtain pose from MoveIt scene
        with self.planning_monitor.read_write() as scene:
            current_state = scene.current_state
            assert isinstance(current_state, RobotState)
            current_state.update(True)
            start_pose=current_state.get_pose(gripper_link)
        request.start_state = robotStateToRobotStateMsg(current_state)
        request.waypoints=[start_pose, pose.pose]
        response= await self.cartesian_plan_client.call_async(request)
        assert isinstance(response, GetCartesianPath.Response)
        plan_result = True if response.fraction>0.9 else False
        robot_trajectory=RobotTrajectory(self.robot_model)
        robot_trajectory.set_robot_trajectory_msg(current_state, response.solution)
        robot_trajectory.joint_model_group_name=selected_arm
        return plan_result, robot_trajectory

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
        plan_result=None,
        trajectory=None,
        vel_factor=1,
        sleep_time=0.0        
        ):
        # execute the plan
        logger.info("Executing plan")
        if plan_result:
            robot_trajectory = plan_result.trajectory
        elif trajectory:
            robot_trajectory = trajectory
        else:
            raise RuntimeError("Provide a motion planning result or a robot trajectory")
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
                execute_result=self.execute(robot,logger, plan_result=plan_result, vel_factor=vel_factor, sleep_time=sleep_time)

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
        self.tiago.shutdown()


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
    pose_goal.header.frame_id = "base_footprint"
    pose_goal.pose.orientation.x = 0.532209038734436
    pose_goal.pose.orientation.y = -0.485505074262619
    pose_goal.pose.orientation.z = -0.4891074001789093
    pose_goal.pose.orientation.w = -0.49174416065216064
    pose_goal.pose.position.x = 0.5485297441482544
    pose_goal.pose.position.y = 0.006088778376579285
    pose_goal.pose.position.z = 0.33060914278030396

    tiago_moveit_py.close_gripper()

    # call tiago method
    logger.info('Moving to goal pose and openning gripper')
    tiago_moveit_py.arm_go_to_pose(pose_goal)

    tiago_moveit_py.open_gripper()





if __name__ == "__main__":
    main()
