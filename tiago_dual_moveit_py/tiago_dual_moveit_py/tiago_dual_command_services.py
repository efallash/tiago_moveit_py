import rclpy
import numpy as np

from tiago_dual_moveit_py.tiago_dual_moveit_py import TiagoDualPy
from tiago_moveit_py_interfaces.srv import ArmControl, GripperControl, TorsoControl
from math import pi, radians

from tiago_dual_moveit_py.service_client import ServiceClientAsync
from tiago_moveit_py_interfaces.srv import GetActuator
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

        self.actuator_client = ServiceClientAsync(
            self, GetActuator, "tiago_dual/get_actuator_characteristic", callback_group=self.cb_client
        )

        self.gripper_relax_offset = 0.001 # Position offset for pressing the gripper fingers against an object

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
            pitch_list = np.linspace(radians(45), radians(90), 3)
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
        success = False
        if request.close:
            self.logger.info(f"DEBUG: Grasp test requested for {arm} gripper")
            # Try to close the gripper
            closing_future = await self.move_gripper(arm, left_finger=0.0, right_finger=0.0, vel_factor=0.1)
            result = await closing_future.get_result_async()

            # Obtain the actuator values after the closing attempt
            actuator_values = await self.get_gripper_actuator_values(arm)
            right_finger_current = abs(actuator_values["right_current"])
            left_finger_current = abs(actuator_values["left_current"])
            right_finger_position = actuator_values["right_position"]
            left_finger_position = actuator_values["left_position"]
            relax_offset = self.gripper_relax_offset    

            #Gripper is completely closed
            if result.result.error_code == 0:
                if right_finger_current < 0.1 and left_finger_current < 0.1:
                    self.logger.info(f"No object detected by {arm} gripper")
                    success = False
                # High current means an small object is detected
                else:
                    self.logger.info(f"Object detected by {arm} gripper")
                    # Relax the gripper so that it does not overheat
                    right_relax_pos = max(right_finger_position - relax_offset, 0.0)
                    left_relax_pos = max(left_finger_position - relax_offset, 0.0)
                    relax_gripper_future = await self.move_gripper(arm, left_finger=left_relax_pos, right_finger=right_relax_pos, vel_factor=0.1)
                    await relax_gripper_future.get_result_async()
                    success = True
            else:
                if right_finger_position > 0.001 or left_finger_position > 0.001:
                    # The gripper is not completely closed
                    
                    right_press_pos = right_finger_position - relax_offset
                    left_press_pos = left_finger_position - relax_offset
                    press_gripper_future = await self.move_gripper(arm, left_finger=left_press_pos, right_finger=right_press_pos, vel_factor=0.1)
                    await press_gripper_future.get_result_async()
                    success = True
                else:
                    self.logger.info(f"No object detected by {arm} gripper")
                    success = False
            
            actuator_values = await self.get_gripper_actuator_values(arm)
            right_finger_current = actuator_values["right_current"]
            left_finger_current = actuator_values["left_current"]
            self.get_logger().info(
                f"Gripper {arm} actuator values after grasp test: "
                f"Right Current: {right_finger_current}, Left Current: {left_finger_current}"
            )
            return success
        else:
            handle = await self.open_gripper(arm)
            result = await handle.get_result_async()
            success = True if result.result.error_code == 0 else False
            # Relax the gripper so that it does not overheat
            actuator_values = await self.get_gripper_actuator_values(arm)
            right_finger_current = abs(actuator_values["right_current"])
            left_finger_current = abs(actuator_values["left_current"])
            right_finger_position = actuator_values["right_position"]
            left_finger_position = actuator_values["left_position"]
            relax_offset = self.gripper_relax_offset    
            right_relax_pos = right_finger_position
            left_relax_pos = left_finger_position
            relax_gripper_future = await self.move_gripper(arm, left_finger=left_relax_pos, right_finger=right_relax_pos, vel_factor=0.1)
            return success


    
    async def get_gripper_actuator_values(self, arm) -> dict:
        name_right = f"gripper_{arm}_right_finger_actuator"
        name_left  = f"gripper_{arm}_left_finger_actuator"

        right_finger_current = await self.actuator_client.send_request_async(
            name=name_right,
            characteristic="current_state"
        )
        left_finger_current = await self.actuator_client.send_request_async(
            name=name_left,
            characteristic="current_state"
        )
        right_finger_position = await self.actuator_client.send_request_async(
            name=name_right,
            characteristic="position_state"
        )
        left_finger_position = await self.actuator_client.send_request_async(
            name=name_left,
            characteristic="position_state"
        )
        return {
            "right_current": right_finger_current.value,
            "left_current": left_finger_current.value,
            "right_position": right_finger_position.value,
            "left_position": left_finger_position.value
        }
    
def main():
    rclpy.init()

    tiago_dual_commander = TiagoDualCommander()

    try:
        rclpy.spin(tiago_dual_commander)
    except KeyboardInterrupt:
        tiago_dual_commander.destroy_node()
    