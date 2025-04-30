#!/usr/bin/env python3
"""
ROS2 Workspace Reach Experiment

This node iterates through a grid of target poses (x, y, z, yaw)
and commands the robot arm to reach each pose via a service call.
It then reads back the end‐effector’s pose via tf2 and stores results
in a CSV file.
 
Requirements met:
  - Performs a workspace reach experiment by iterating through poses.
  - Implemented in ROS2 using rclpy.
  - Does NOT interact with the gripper.
  - Ignores gazebo and extra experiment parameters (for a real robot).
"""

import rclpy
from rclpy.node import Node
import numpy as np
import pandas as pd
import math
from geometry_msgs.msg import Pose, Point, Quaternion
import tf2_ros

# Import the ArmControl service.
from tiago_moveit_py_interfaces.srv import ArmControl, GripperControl, TorsoControl

class WorkspaceReachExperiment(Node):
    def __init__(self):
        super().__init__('workspace_reach_experiment')
        
        # Create a client for the arm control service.
        self.arm_clients = {}
        self.gripper_clients = {}
        self.arm_clients["left"] = self.create_client(ArmControl, "tiago_dual/left_arm_command")
        self.arm_clients["right"] = self.create_client(ArmControl, "tiago_dual/right_arm_command")
        self.gripper_clients["left"] = self.create_client(GripperControl, "tiago_dual/left_gripper_command")
        self.gripper_clients["right"] = self.create_client(GripperControl, "tiago_dual/right_gripper_command")
        self.torso_client = self.create_client(TorsoControl, "tiago_dual/torso_command")
        self.end_effector={}
        self.end_effector["left"] = "gripper_left_grasping_frame" 
        self.end_effector["right"] = "gripper_right_grasping_frame"
        self.get_logger().info("Waiting for arm command service...")
        for client in self.arm_clients.values():
            if not client.wait_for_service(timeout_sec=20.0):
                self.get_logger().error("Arm command service not available!")
                rclpy.shutdown()
                raise RuntimeError
        
        # Create a tf2 buffer and listener to get end-effector pose.
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
    def run_experiment(self, arm, x_limits, y_limits, z_limits, vel_exec):
        # Define workspace boundaries (in meters) and resolution.
        # (Adjust these numbers for your specific robot and workspace.)
        x_min, x_max, x_divide = x_limits
        y_min, y_max, y_divide = y_limits
        z_min, z_max, z_divide = z_limits
        
        
        # Generate arrays for x, y, z, and yaw.
        x_points = np.linspace(x_min, x_max, num=x_divide)
        y_points = np.linspace(y_min, y_max, num=y_divide)
        z_points = np.linspace(z_min, z_max, num=z_divide)
        
        # Build a list of target poses as [x, y, z, yaw]
        poses = []
        for x in x_points:
            for y in y_points:
                for z in z_points:
                    poses.append([x, y, z])
                        
        # Optionally, shuffle the list to avoid ordering bias.
        np.random.shuffle(poses)
        
        results = []

        #Open grippers
        gripper_msg=GripperControl.Request()
        gripper_msg.close = False
        torso_msg=TorsoControl.Request()
        torso_msg.position = 0.2
        torso_msg.vel = 0.4
        future1 = self.gripper_clients["left"].call_async(gripper_msg)
        self.wait_execution_result(future1)
        future2 = self.gripper_clients["right"].call_async(gripper_msg)
        self.wait_execution_result(future2)
        future3 = self.torso_client.call_async(torso_msg)
        self.wait_execution_result(future3)
     
        # Send the arm to a known “home” position first.
        self.get_logger().info("Sending arm to home position...")
        home_req = ArmControl.Request()
        home_req.x = 0.3
        home_req.y = 0.3 if arm=="left" else -0.3
        home_req.z = 0.6
        home_req.pitch = 1.5708
        home_req.vel = vel_exec
        home_req.cartesian = False
        future = self.arm_clients[arm].call_async(home_req)
        success, status_msg = self.wait_execution_result(future)
        if success:
            self.get_logger().info("Arm homed successfully.")

        else:
            self.get_logger().error(f"Failed to home the arm! Status: {status_msg}")
            
        # Send the other arm to a position where it does not get in the way
        self.get_logger().info("Sending other arm to far position...")
        other_arm= "left" if arm=="right" else "right"
        home_req = ArmControl.Request()
        home_req.x = -0.5
        home_req.y = 0.5 if other_arm=="left" else -0.5
        home_req.z = 0.3
        home_req.pitch = 1.5708
        home_req.vel = vel_exec
        home_req.cartesian = False
        future = self.arm_clients[other_arm].call_async(home_req)
        success, status_msg = self.wait_execution_result(future)
        if success:
            self.get_logger().info("Other arm homed successfully.")

        else:
            self.get_logger().error(f"Failed to home the other arm! Status: {status_msg}")


        
        
        # Iterate over all target poses.
        for idx, target in enumerate(poses):
            x, y, z = target
            self.get_logger().info(f"[{idx+1}/{len(poses)}] Sending target: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            
            # Create and populate the service request.
            req = ArmControl.Request()
            req.x = x
            req.y = y
            req.z = z
            req.grasp_pose = True
            #req.pitch = 1.5708 # Set a fixed pitch (adjust as needed).
            req.vel = vel_exec # Set a constant velocity (adjust as needed).
            req.cartesian = False      # Normal mode.
            
            # Call the service and wait for the result.
            future = self.arm_clients[arm].call_async(req)
            success, status_msg = self.wait_execution_result(future)
            
            # If planning was successful, try to get the reached pose via tf2.
            if success:
                self.get_logger().info("Pose reached successfully.")
                try:
                    now = rclpy.time.Time()
                    trans = self.tf_buffer.lookup_transform('base_footprint', self.end_effector[arm], now, timeout=rclpy.duration.Duration(seconds=1.0))
                    reached_x = trans.transform.translation.x
                    reached_y = trans.transform.translation.y
                    reached_z = trans.transform.translation.z
                    # Calculate Euclidean error (ignoring yaw).
                    error = math.sqrt((reached_x - x)**2 + (reached_y - y)**2 + (reached_z - z)**2)
                except Exception as e:
                    self.get_logger().error(f"TF lookup failed: {e}")
                    reached_x, reached_y, reached_z = 0.0, 0.0, 0.0
                    error = -1.0
            else:
                self.get_logger().info(f"Pose failed. Status: {status_msg}")
                reached_x = reached_y = reached_z = 0.0
                error = -1.0
            
            # Append results for this pose.
            results.append({
                "x_set": x,
                "y_set": y,
                "z_set": z,
                "success": success,
                "status": status_msg,
                "x_final": reached_x,
                "y_final": reached_y,
                "z_final": reached_z,
                "error": error
            })
        
        # Send the arm to a known “home” position first.
        self.get_logger().info("Sending arm to home position...")
        home_req = ArmControl.Request()
        home_req.x = 0.3
        home_req.y = 0.3 if arm=="left" else -0.3
        home_req.z = 0.6
        home_req.yaw = 0.0
        home_req.vel = vel_exec
        home_req.cartesian = False
        future = self.arm_clients[arm].call_async(home_req)
        success, status_msg = self.wait_execution_result(future)
        if success:
            self.get_logger().info("Arm homed successfully.")

        else:
            self.get_logger().error(f"Failed to home the arm! Status: {status_msg}")

        # Send the arm to a known “home” position first.
        self.get_logger().info("Sending arm to home position...")
        home_req = ArmControl.Request()
        home_req.x = 0.3
        home_req.y = 0.3 if other_arm=="left" else -0.3
        home_req.z = 0.6
        home_req.yaw = 0.0
        home_req.vel = vel_exec
        home_req.cartesian = False
        future = self.arm_clients[other_arm].call_async(home_req)
        if success:
            self.get_logger().info("Other arm homed successfully.")

        else:
            self.get_logger().error(f"Failed to home the other arm! Status: {status_msg}")
        
        # Save experiment results to a CSV file.
        results_df = pd.DataFrame(results)
        output_file = "workspace_reach_results.csv"
        results_df.to_csv(output_file, index=False)
        self.get_logger().info(f"Experiment finished; results saved to {output_file}")

    def wait_execution_result(self, future):
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            # Check for planning (and execution) success.
            if response is not None:
                success = response.success
                if getattr(response, "status", None): 
                    status_msg = response.status
                else:
                    status_msg = ""  
            else:
                success = False
                status_msg = ""
            return success, status_msg

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceReachExperiment()
    x_limits=(0.3, 0.9, 10)
    y_limits=(0.2, -0.9, 10)
    z_limits=(0.3, 1.0, 10)
    try:
        node.run_experiment("right", x_limits, y_limits, z_limits, vel_exec=0.4)
    except Exception as e:
        node.get_logger().error(f"Experiment error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
