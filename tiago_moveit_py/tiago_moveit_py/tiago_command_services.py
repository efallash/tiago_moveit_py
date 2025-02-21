import rclpy
import numpy as np

from tiago_moveit_py.tiago_moveit_py import TiagoPy
from tiago_moveit_py_interfaces.srv import ArmControl, GripperControl, TorsoControl
from math import pi

from tf_transformations import quaternion_from_euler
from moveit.planning import PlanRequestParameters, PlanningComponent
from geometry_msgs.msg import PoseStamped

class TiagoCommander(TiagoPy):
    def __init__(self, name='tiago_commander'):
        super().__init__(name)
        # Service Servers
        self.srv_left_arm = self.create_service(
            ArmControl, "tiago/arm_command", self.arm_callback, callback_group=self.cb_server
        )
        self.srv_right_gripper = self.create_service(
            GripperControl, "tiago/gripper_command", self.gripper_callback, callback_group=self.cb_server
        )
        #self.srv_torso = self.create_service(
        #    TorsoControl, "tiago_dual/torso_command", self.torso_callback, callback_group=self.cb_server
        #)
        self.get_logger().info("Tiago Command Services are now available.")


    async def arm_callback(
        self, request: ArmControl.Request, response: ArmControl.Response
    ):
        self.get_logger().info(f"Left arm command received (x= {request.x}, y= {request.y}, z= {request.z}, vel= {request.vel}, pose= {request.named_pose})")
        success, status = await self.arm_command(request)
        response.success = success
        response.status = status
        self.get_logger().info(f"Left arm command response (success= {success}, status= {status})")
        return response

    async def gripper_callback(
        self, request: GripperControl.Request, response: GripperControl.Response
    ):
        self.get_logger().info("Right gripper command received")
        success = await self.gripper_control(request)
        response.success = success
        return response
    
    def torso_callback(self, request: TorsoControl.Request, response: TorsoControl.Response):
        self.get_logger().info("Torso command received")
        success, status = self.torso_go_to_position(request.position, vel_factor=request.vel)
        response.success = success
        response.status = status
        return response

    async def arm_command(self, request: ArmControl.Request):
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
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_footprint"
        target_pose.pose.position.x = request.x
        target_pose.pose.position.y = request.y
        target_pose.pose.position.z = request.z
        roll=0
        pitch=1.5708
        yaw=request.yaw
        orient = quaternion_from_euler(roll, pitch, yaw)
        self.logger.info(f"euler: (r: {roll}, p: {pitch}, y: {yaw})")
        self.logger.info(f"orientation: {orient}")
        target_pose.pose.orientation.x = orient[0]
        target_pose.pose.orientation.y = orient[1]
        target_pose.pose.orientation.z = orient[2]
        target_pose.pose.orientation.w = orient[3]

        if request.cartesian:
            success, status = await self.arm_go_to_pose_cartesian(target_pose, request.vel)
        else:
            success, status = self.arm_go_to_pose(target_pose, request.vel)

        return (success, status)
    
    async def gripper_control(self, request: GripperControl.Request):
        if request.close:
            future = self.close_gripper()
        else:
            future = self.open_gripper()
        self.logger.info(f"DEBUG - OBTAINED ACTION FUTURE")
        handle = await future
        self.logger.info(f"DEBUG - OBTAINED HANDLE")
        result = await handle.get_result_async()
        self.logger.info(f"DEBUG - OBTAINED RESULT: {result}")
        success = True if result.result.error_code == 0 else False
        return success
    
def main():
    rclpy.init()

    tiago_commander = TiagoCommander()

    try:
        rclpy.spin(tiago_commander)
    except KeyboardInterrupt:
        tiago_commander.destroy_node()
    