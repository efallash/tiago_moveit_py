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
from tiago_moveit_py_interfaces.srv import ArmControl  # adjust as needed

class WorkspaceReachExperiment(Node):
    def __init__(self):
        super().__init__('workspace_reach_experiment')
        
        # Create a client for the arm control service.
        self.arm_clients={}
        self.arm_clients["left"] = self.create_client(ArmControl, "tiago_dual/left_arm_command")
        self.arm_clients["right"] = self.create_client(ArmControl, "tiago_dual/right_arm_command")
        self.end_effector={}
        self.end_effector["left"] = "gripper_left_grasping_frame" 
        self.end_effector["right"] = "gripper_right_grasping_frame"
        self.get_logger().info("Waiting for arm command service...")
        for client in self.arm_clients.values():
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("Arm command service not available!")
                rclpy.shutdown()
                raise RuntimeError
        
        # Create a tf2 buffer and listener to get end-effector pose.
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
    def run_experiment(self, arm, x_limits, y_limits, z_limits, yaw_limits, vel_exec):
        # Define workspace boundaries (in meters) and resolution.
        # (Adjust these numbers for your specific robot and workspace.)
        x_min, x_max, x_divide = x_limits
        y_min, y_max, y_divide = y_limits
        z_min, z_max, z_divide = z_limits
        
        # Define yaw limits (in radians) and resolution.
        yaw_min, yaw_max, yaw_divide = yaw_limits
        
        # Generate arrays for x, y, z, and yaw.
        x_points = np.linspace(x_min, x_max, num=x_divide)
        y_points = np.linspace(y_min, y_max, num=y_divide)
        z_points = np.linspace(z_min, z_max, num=z_divide)
        yaw_points = np.linspace(yaw_min, yaw_max, num=yaw_divide)
        
        # Build a list of target poses as [x, y, z, yaw]
        poses = []
        for x in x_points:
            for y in y_points:
                for z in z_points:
                    for yaw in yaw_points:
                        poses.append([x, y, z, yaw])
                        
        # Optionally, shuffle the list to avoid ordering bias.
        np.random.shuffle(poses)
        
        results = []
        
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
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Failed to home the arm!")
        else:
            self.get_logger().info("Arm homed successfully.")

        # Send the other arm to a position where it does not get in the way
        self.get_logger().info("Sending other arm to far position...")
        home_req = ArmControl.Request()
        home_req.x = -0.5
        home_req.y = 0.5 if arm=="left" else -0.5
        home_req.z = 0.6
        home_req.yaw = 0.0
        home_req.vel = vel_exec
        home_req.cartesian = False
        other_arm= "left" if arm=="right" else "right"
        future = self.arm_clients[other_arm].call_async(home_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Failed to move the other arm!")
        else:
            self.get_logger().info("Other arm homed successfully.")


        
        
        # Iterate over all target poses.
        for idx, target in enumerate(poses):
            x, y, z, yaw = target
            self.get_logger().info(f"[{idx+1}/{len(poses)}] Sending target: x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}")
            
            # Create and populate the service request.
            req = ArmControl.Request()
            req.x = x
            req.y = y
            req.z = z
            req.yaw = yaw      # New field for yaw.
            req.vel = vel_exec # Set a constant velocity (adjust as needed).
            req.cartesian = False      # Normal mode.
            
            # Call the service and wait for the result.
            future = self.arm_clients[arm].call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            # Check for planning (and execution) success.
            if response is not None:
                plan_success = response.success 
                exec_success = response.status  
            else:
                plan_success = False
                exec_success = ""
            
            # If planning was successful, try to get the reached pose via tf2.
            if plan_success:
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
                reached_x = reached_y = reached_z = 0.0
                error = -1.0
            
            # Append results for this pose.
            results.append({
                "x_set": x,
                "y_set": y,
                "z_set": z,
                "yaw_set": yaw,
                "plan_success": plan_success,
                "exec_success": exec_success,
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
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Failed to home the arm!")
        else:
            self.get_logger().info("Arm homed successfully.")

        # Send the arm to a known “home” position first.
        self.get_logger().info("Sending arm to home position...")
        home_req = ArmControl.Request()
        home_req.x = 0.3
        home_req.y = 0.3 if arm=="left" else -0.3
        home_req.z = 0.6
        home_req.yaw = 0.0
        home_req.vel = vel_exec
        home_req.cartesian = False
        future = self.arm_clients[other_arm].call_async(home_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Failed to home the arm!")
        else:
            self.get_logger().info("Arm homed successfully.")
        
        # Save experiment results to a CSV file.
        results_df = pd.DataFrame(results)
        output_file = "workspace_reach_results.csv"
        results_df.to_csv(output_file, index=False)
        self.get_logger().info(f"Experiment finished; results saved to {output_file}")

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceReachExperiment()
    x_limits=(0.3, 0.9, 5)
    y_limits=(-0.2, 0.9, 5)
    z_limits=(0.3, 0.9, 5)
    yaw_limits=(-math.pi/2, math.pi/2, 5)
    try:
        node.run_experiment("left", x_limits, y_limits, z_limits, yaw_limits, vel_exec=0.4)
    except Exception as e:
        node.get_logger().error(f"Experiment error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
