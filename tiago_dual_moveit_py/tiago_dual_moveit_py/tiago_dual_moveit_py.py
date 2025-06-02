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
from tf_transformations import euler_from_quaternion, quaternion_matrix
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class TiagoDualPy(Node):
    def __init__(self, name='moveit_py'):
        super().__init__(name)
        self.logger = get_logger(f'tiago_dual.{name}')
        #Create callback groups
        self.cb_client = MutuallyExclusiveCallbackGroup()
        self.cb_internal_client = MutuallyExclusiveCallbackGroup()
        self.cb_server = MutuallyExclusiveCallbackGroup()
        self.cb_topic = MutuallyExclusiveCallbackGroup()

        # instantiate MoveItPy instance and get planning component
        self.tiago_dual = MoveItPy(node_name=name)
        trajectory_execution = self.tiago_dual.get_trajectory_execution_manager()
        self.robot_model = self.tiago_dual.get_robot_model()
        self.planning_monitor=self.tiago_dual.get_planning_scene_monitor()
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
        self.gripper_links = {}
        self.planning_links = {}
        self.grippers = {}

        # Populate arms and grippers
        self.groups['right'] = self.tiago_dual.get_planning_component("arm_right")
        self.groups['left'] = self.tiago_dual.get_planning_component("arm_left")
        self.groups['right_torso'] = self.tiago_dual.get_planning_component("arm_right_torso")
        self.groups['left_torso'] = self.tiago_dual.get_planning_component("arm_left_torso")
        self.groups['both_arms_torso'] = self.tiago_dual.get_planning_component("both_arms_torso")
        self.groups['torso'] = self.tiago_dual.get_planning_component("torso")
        self.grippers["left"] = ActionClient(
            self, FollowJointTrajectory, "/gripper_left_controller/follow_joint_trajectory", callback_group=self.cb_client)
        self.grippers["right"] = ActionClient(
            self, FollowJointTrajectory, "/gripper_right_controller/follow_joint_trajectory", callback_group=self.cb_client)
        self.gripper_links['right'] = 'custom_gripper_right_grasping_frame'
        self.gripper_links['left'] = 'custom_gripper_left_grasping_frame'
        self.planning_links['right'] = 'gripper_right_grasping_frame'
        self.planning_links['left'] = 'gripper_left_grasping_frame'

        self.publish_grasping_frame()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.logger.info("MoveItPy instance created")

    def publish_grasping_frame(self):
        # Create a static transform broadcaster
        self.tf_right_broadcaster = StaticTransformBroadcaster(self)
        self.tf_left_broadcaster = StaticTransformBroadcaster(self)
        # Create a static transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "gripper_right_grasping_frame"
        transform.child_frame_id = "custom_gripper_right_grasping_frame"
        transform.transform.translation.x = 0.08
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        # Send the transformation
        self.tf_right_broadcaster.sendTransform(transform)

        transform.header.frame_id = "gripper_left_grasping_frame"
        transform.child_frame_id = "custom_gripper_left_grasping_frame"

        self.tf_left_broadcaster.sendTransform(transform)
        self.logger.info("Grasping frames published")
    
    def transform_pose_to_planning_frame(self, pose: PoseStamped, arm: str) -> PoseStamped:
        """Transform a pose defined for custom frame to be used with the actual planning frame

        Note: The custom frame must have a static transform to the planning frame, AND, the same orientation.
        
        Using TF2's built-in pose transformation utilities.
        
        Args:
            pose: The pose to transform
            arm: 'left' or 'right' arm
            
        Returns:
            The transformed pose for use with the planning frame
        """
        if arm not in ['left', 'right']:
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
            return pose
    
        # Transform this identity pose to the base frame (typically base_footprint)
        # This gives us the pose of the custom frame in the base frame
        try:      
            # Now get transform from planning to custom frame
            planning_to_custom_transform = self.tf_buffer.lookup_transform(
                self.gripper_links[arm],    # custom frame
                self.planning_links[arm],   # planning frame
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=10.0)
            )
            
            # Extract transformation components
            dx = planning_to_custom_transform.transform.translation.x
            dy = planning_to_custom_transform.transform.translation.y
            dz = planning_to_custom_transform.transform.translation.z
            
            # Calculate the offset in the world frame based on the pose's orientation
            q = [pose.pose.orientation.x, pose.pose.orientation.y, 
                pose.pose.orientation.z, pose.pose.orientation.w]
            rot_matrix = quaternion_matrix(q)
            
            # Apply rotation to the offset
            offset = [dx, dy, dz, 1.0]
            offset_in_world = rot_matrix.dot(offset)
            
            # Apply the offset to the original pose
            adjusted_pose = PoseStamped()
            adjusted_pose.header = pose.header
            adjusted_pose.pose = pose.pose
            adjusted_pose.pose.position.x += offset_in_world[0]
            adjusted_pose.pose.position.y += offset_in_world[1]
            adjusted_pose.pose.position.z += offset_in_world[2]
            
            self.logger.info(f"Transformed pose using TF2 with offset {[dx, dy, dz]}")
            return adjusted_pose
            
        except Exception as e:
            self.logger.error(f"TF Error: {e}")
            # Return original pose if transformation fails
            return None 

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

    def close_gripper(self, arm, vel_factor = 0.1, sleep_time=0.1):
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        self.logger.info("Closing gripper")
        future = self.gripper_action(arm, point, vel_factor, sleep_time)
        return future

    def open_gripper(self, arm, vel_factor = 0.1, sleep_time=0.1):
        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        self.logger.info("Opening gripper")
        future = self.gripper_action(arm, point, vel_factor, sleep_time)
        return future

    def move_gripper (self, arm, left_finger, right_finger, vel_factor = 0.1, sleep_time = 0.1):
        point = JointTrajectoryPoint()
        point.positions = [right_finger, left_finger]
        self.logger.info(f"Moving {arm} gripper to: {right_finger},{left_finger}")
        future = self.gripper_action(arm, point, vel_factor, sleep_time)
        return future
    
    def gripper_action(self, arm, point:JointTrajectoryPoint, vel_factor, sleep_time):
        if arm == 'left' or arm == 'right':
            msg = FollowJointTrajectory.Goal()
            action_client = self.grippers[arm]
        else:
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
            return False
        
        with self.planning_monitor.read_write() as scene:
            current_state = scene.current_state
            assert isinstance(current_state, RobotState)
            current_state.update(True)
            joint_positions=current_state.get_joint_group_positions(f"gripper_{arm}")

        joint_goals = list(point.positions)
        distances = [abs(joint_positions[0]-joint_goals[0]),abs(joint_positions[1]-joint_goals[1])]
        duration = (max(distances)/vel_factor)*2
        point.time_from_start.sec = int(duration)
        
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.trajectory.joint_names = [f"gripper_{arm}_right_finger_joint", f"gripper_{arm}_left_finger_joint"]
        msg.trajectory.points = [point]
        self.logger.debug(f"DEBUG - Sending goal {point} with duration {duration}")
        future = action_client.send_goal_async(msg)
        time.sleep(duration + sleep_time)
        return future

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
        assert isinstance(both_arms_torso, PlanningComponent)
        pose_right=self.transform_pose_to_planning_frame(pose_right, arm='right')
        pose_left=self.transform_pose_to_planning_frame(pose_left, arm='left')

        if pose_left is None or pose_right is None:
            return (False, 'PLAN_FAILED')
        
        with self.planning_monitor.read_write() as scene:
            current_state = scene.current_state
            assert isinstance(current_state, RobotState)
            robot_state = RobotState(self.robot_model) 
            current_state.update(True)

            robot_state.set_joint_group_positions("torso", [torso_constraint])
            robot_state.update(True)

            self.logger.info("Planning right arm")
            robot_state.set_from_ik("arm_right", pose_right.pose, self.planning_links["right"], 1.0)
            self.logger.info("Planning left arm")
            robot_state.set_from_ik("arm_left", pose_left.pose, self.planning_links["left"], 1.0)
            robot_state.update(True) 
        
        both_arms_torso.set_start_state_to_current_state()
        both_arms_torso.set_goal_state(robot_state=robot_state)
        return self.plan_and_execute(self.tiago_dual, both_arms_torso, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)

    def arm_go_to_pose(self, arm: str, pose: PoseStamped, vel_factor=0.2, sleep_time=0.1):
        if arm=='left' or arm=='right':
            selected_arm=self.groups[arm]
            planning_link=self.planning_links[arm]
        else:
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
            return (False, 'PLAN_FAILED')
        pose=self.transform_pose_to_planning_frame(pose, arm=arm)
        assert isinstance(selected_arm, PlanningComponent)
        selected_arm.set_start_state_to_current_state()
        selected_arm.set_goal_state(pose_stamped_msg=pose, pose_link=planning_link)
        angles=euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        self.logger.info(f'Moving to pose x: {pose.pose.position.x} y: {pose.pose.position.y} z: {pose.pose.position.z} r: {angles[0]} p: {angles[1]} y: {angles[2]}')
        return self.plan_and_execute(self.tiago_dual, selected_arm, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)
    
    def torso_go_to_position(self, position: float, vel_factor=0.2, sleep_time=0.1):
        torso=self.groups["torso"]
        assert isinstance(torso, PlanningComponent)
        robot_state = RobotState(self.robot_model) 
        robot_state.set_joint_group_positions("torso", [position])
        robot_state.update(True)
        torso.set_goal_state(robot_state=robot_state)
        self.logger.info(f"Moving torso to position: {position} m")
        return self.plan_and_execute(self.tiago_dual, torso, self.logger, vel_factor=vel_factor, sleep_time=sleep_time)
    
    async def arm_go_to_pose_cartesian(self, arm: str, pose: PoseStamped, vel_factor=0.2, sleep_time=0.1):
        self.logger.info('Planning and executing cartesian path')
        plan_result, robot_trajectory =await self.cartesian_plan(arm, pose)
        if plan_result:
            if vel_factor>0:
                execute_result=self.execute(self.tiago_dual, self.logger, trajectory=robot_trajectory, vel_factor=vel_factor, sleep_time=sleep_time)

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
        if arm=='left' or arm=='right':
            selected_arm=self.groups[arm].planning_group_name
            planning_link=self.planning_links[arm]
        else:
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
            return False
        pose=self.transform_pose_to_planning_frame(pose, arm=arm)
        #Define request message
        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = pose.header.frame_id
        request.link_name = planning_link
        request.group_name = selected_arm
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        #Obtain pose from MoveIt scene
        with self.planning_monitor.read_write() as scene:
            current_state = scene.current_state
            assert isinstance(current_state, RobotState)
            current_state.update(True)
            start_pose=current_state.get_pose(planning_link)
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
    tiago_dual.arm_go_to_pose('right', pose_goal, vel_factor=0.2)

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
    tiago_dual.arm_go_to_pose('left', pose_goal, vel_factor=0.2)


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
    tiago_dual.arm_go_to_pose('right', pose_goal, vel_factor=0.2)



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
    tiago_dual.arm_go_to_pose('left', pose_goal, vel_factor=0.2)


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
    tiago_dual.dual_arm_go_to_pose(pose_goal_r, pose_goal_l, 0.1, vel_factor=0.2)

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
    tiago_dual.dual_arm_go_to_pose(pose_goal_r, pose_goal_l, 0.3, vel_factor=0.2)





if __name__ == "__main__":
    main()
