import rclpy
import numpy as np

from tiago_dual_moveit_py.tiago_dual_moveit_py import TiagoDualPy
from tiago_moveit_py_interfaces.srv import ArmControl, GripperControl, TorsoControl
from math import pi, radians

from tf_transformations import quaternion_from_euler
from moveit.planning import PlanRequestParameters, PlanningComponent
from geometry_msgs.msg import PoseStamped

class TiagoDualCommander(TiagoDualPy):
    def __init__(self, name='tiago_dual_commander'):
        super().__init__(name)
        # Service Servers
        self.srv_left_arm = self.create_service(
            ArmControl, "tiago_dual/left_arm_command", self.left_arm_callback, callback_group=self.cb_server
        )
        self.srv_right_arm = self.create_service(
            ArmControl, "tiago_dual/right_arm_command", self.right_arm_callback, callback_group=self.cb_server
        )
        self.srv_left_gripper = self.create_service(
            GripperControl, "tiago_dual/left_gripper_command", self.left_gripper_callback, callback_group=self.cb_server
        )
        self.srv_right_gripper = self.create_service(
            GripperControl, "tiago_dual/right_gripper_command", self.right_gripper_callback, callback_group=self.cb_server
        )
        self.srv_torso = self.create_service(
            TorsoControl, "tiago_dual/torso_command", self.torso_callback, callback_group=self.cb_server
        )
        self.get_logger().info("Tiago Command Services are now available.")


    async def left_arm_callback(
        self, request: ArmControl.Request, response: ArmControl.Response
    ):
        self.get_logger().info(f"Left arm command received (x= {request.x}, y= {request.y}, z= {request.z}, vel= {request.vel}, pose= {request.named_pose})")
        success, status = await self.arm_command(request, arm="left")
        response.success = success
        response.status = status
        self.get_logger().info(f"Left arm command response (success= {success}, status= {status})")
        return response

    async def right_arm_callback(
        self, request: ArmControl.Request, response: ArmControl.Response
    ):
        self.get_logger().info(f"Right arm command received (x= {request.x}, y= {request.y}, z= {request.z}, vel= {request.vel}, pose= {request.named_pose})")
        success, status = await self.arm_command(request, arm="right")
        response.success = success
        response.status = status
        self.get_logger().info(f"Right arm command response (success= {success}, status= {status})")
        return response   
    
    async def left_gripper_callback(
        self, request: GripperControl.Request, response: GripperControl.Response
    ):
        self.get_logger().info("Left gripper command received")
        success = await self.gripper_control(request, "left")
        response.success = success
        return response

    async def right_gripper_callback(
        self, request: GripperControl.Request, response: GripperControl.Response
    ):
        self.get_logger().info("Right gripper command received")
        success = await self.gripper_control(request, "right")
        response.success = success
        return response
    
    def torso_callback(self, request: TorsoControl.Request, response: TorsoControl.Response):
        self.get_logger().info("Torso command received")
        success, status = self.torso_go_to_position(request.position, vel_factor=request.vel)
        response.success = success
        response.status = status
        return response

    async def arm_command(self, request: ArmControl.Request, arm):
        # Check if velocity command is in allowed values
        if request.vel > 0 and request.vel <= 1:
            self.get_logger().info("Executing requested command")
            execute = True  # Flag for execution
        else:
            self.get_logger().info("Planning requested command")
            execute = False

        # If a named pose is commanded
        if len(request.named_pose) > 0:
            success, status = self.arm_go_to_named_pose(
                arm, request.named_pose, request.vel
            )

            if success:
                if execute:
                    self.get_logger().info(f"Executed pose. Status code: {status}")
                else:
                    self.get_logger().info(
                        f"Pose not executed. Planning code: {status}"
                    )
            else:
                self.get_logger().error(f"Pose failed. Error code: {status}")

            return (success, status)


        # Define pose object
        orient = self.obtain_orientation(arm, request)
        if orient is None:
            self.get_logger().error("Orientation not obtained")
            return (False, 'PLAN_FAILED')
        
        target_pose = self.pose_to_msg(request, orient)

        if request.cartesian:
            success, status = await self.arm_go_to_pose_cartesian(arm, target_pose, request.vel)
        else:
            success, status = self.arm_go_to_pose(arm, target_pose, request.vel)

        return (success, status)

    def obtain_orientation(self, arm, request):
        if request.grasp_pose:
            roll = request.roll
            pitch_list = np.linspace(radians(30), radians(60), 3)
            if arm == "left":
                yaw_list = np.linspace(radians(-90), radians(90), 10)
            else:
                yaw_list = np.linspace(radians(90), radians(-90), 10)
            
            for pitch in pitch_list:
                for yaw in yaw_list:
                    orient = quaternion_from_euler(yaw, pitch, roll, axes="rzyx")
                    target_pose = self.pose_to_msg(request, orient)
                    success, status = self.arm_go_to_pose(arm, target_pose, 0.0)
                    if success:
                        self.logger.info(f"euler: (r: {roll}, p: {pitch}, y: {yaw}, axes: rzyx)")
                        self.logger.info(f"orientation: {orient}")
                        return orient

        else:
            roll = request.roll
            pitch = request.pitch
            yaw = request.yaw
            orient = quaternion_from_euler(yaw, pitch, roll, axes="rzyx")
            self.logger.info(f"euler: (r: {roll}, p: {pitch}, y: {yaw}, axes: rzyx)")
            self.logger.info(f"orientation: {orient}")
            return orient
    
    def pose_to_msg(self, request, orient):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_footprint"
        target_pose.pose.position.x = request.x
        target_pose.pose.position.y = request.y
        target_pose.pose.position.z = request.z
        target_pose.pose.orientation.x = orient[0]
        target_pose.pose.orientation.y = orient[1]
        target_pose.pose.orientation.z = orient[2]
        target_pose.pose.orientation.w = orient[3]
        return target_pose
    
    async def gripper_control(self, request: GripperControl.Request, arm):
        if request.close:
            future = self.close_gripper(arm)
        else:
            future = self.open_gripper(arm)
        self.logger.info(f"DEBUG - OBTAINED ACTION FUTURE")
        handle = await future
        self.logger.info(f"DEBUG - OBTAINED HANDLE")
        result = await handle.get_result_async()
        self.logger.info(f"DEBUG - OBTAINED RESULT: {result}")
        success = True if result.result.error_code == 0 else False
        return success
    
def main():
    rclpy.init()

    tiago_dual_commander = TiagoDualCommander()

    try:
        rclpy.spin(tiago_dual_commander)
    except KeyboardInterrupt:
        tiago_dual_commander.destroy_node()
    