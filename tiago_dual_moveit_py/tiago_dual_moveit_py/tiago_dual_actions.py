import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Point

from tiago_dual_moveit_py.tiago_dual_moveit_py import TiagoDualPy
from tf2_ros import Buffer, TransformListener
from tiago_moveit_py_interfaces.srv import ArmControl, GripperControl, TorsoControl, Trigger

class TiagoDualActions(TiagoDualPy):
    def __init__(self, name='tiago_dual_actions_moveit_py'):
        super().__init__(name)
        

    def change_hands(self, giver_arm, receiver_arm, vel_factor = 0.2, sleep_time=0.1):
        if (giver_arm != 'right' or giver_arm != 'left') or (receiver_arm != 'right' or receiver_arm != 'left'):
            self.logger.error('Wrong Arm Selected: Arm must be "left" or "right".')
        else:
            self.logger.info("Going to change hands position...")
            self.arm_go_to_named_pose(arm=giver_arm, pose_name='pre-give', vel_factor=vel_factor, sleep_time=sleep_time)
            self.arm_go_to_named_pose(arm=receiver_arm, pose_name='pre-collect', vel_factor=vel_factor, sleep_time=sleep_time)
            self.open_gripper(arm=receiver_arm, vel_factor=vel_factor, sleep_time=sleep_time)

            self.logger.info("Changing hands...")
            self.arm_go_to_named_pose(arm=giver_arm, pose_name='give', vel_factor=vel_factor, sleep_time=sleep_time)
            self.arm_go_to_named_pose(arm=receiver_arm, pose_name='collect', vel_factor=vel_factor, sleep_time=vel_factor)

            self.close_gripper(receiver_arm, vel_factor=vel_factor, sleep_time=sleep_time)
            self.open_gripper(giver_arm, vel_factor=vel_factor, sleep_time=sleep_time)

            self.arm_go_to_named_pose(arm=giver_arm, pose_name='pre-give', vel_factor=vel_factor, sleep_time=sleep_time)

            self.arm_go_to_named_pose(arm=receiver_arm, pose_name='pre-collect', vel_factor=vel_factor, sleep_time=sleep_time)
            self.logger.info("Hands changed.")


class TiagoDualGraspingTest(Node):
    def __init__(self):
        super().__init__('tiago_dual_grasping_test')
        self.enabled = False

        #Default positions:
        self.approach_z = 1.1
        self.home_z = 1.0
        self.home_x = 0.2
        self.home_y = 0.3
        self.slow_vel = 0.2
        self.fast_vel = 0.5
        self.torso_position = 0.34

        self.client_cb = MutuallyExclusiveCallbackGroup()
        self.server_cb = MutuallyExclusiveCallbackGroup()

        self.right_arm_client = ServiceClientAsync(self, ArmControl, "tiago_dual/right_arm_command", self.client_cb)
        self.left_arm_client = ServiceClientAsync(self, ArmControl, "tiago_dual/left_arm_command", self.client_cb)
        self.right_gripper_client = ServiceClientAsync(self, GripperControl, "tiago_dual/right_gripper_command", self.client_cb)
        self.left_gripper_client = ServiceClientAsync(self, GripperControl, "tiago_dual/left_gripper_command", self.client_cb)
        self.torso_client = ServiceClientAsync(self, TorsoControl, "tiago_dual/torso_command", self.client_cb)

        self.startup_srv = self.create_service(Trigger, "tiago_dual/enable_grasping", self.startup_callback, callback_group=self.server_cb)
        self.disable_srv = self.create_service(Trigger, "tiago_dual/disable_grasping", self.disable_callback, callback_group=self.server_cb)
        self.pick_srv = self.create_service(ArmControl, "tiago_dual/pick", self.pick_callback, callback_group=self.server_cb)
        self.place_srv = self.create_service(ArmControl, "tiago_dual/place", self.place_callback, callback_group=self.server_cb)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Tiago Dual Grasping Test Node Initialized.")        

    async def startup_callback(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Startup command received.")
        if not self.enabled:
            self.enabled = True
            x = self.home_x
            z = self.home_z
            vel = self.fast_vel
            y_left = self.home_y
            y_right = -self.home_y
            torso_height = self.torso_position

            # Move the torso to the desired position
            call_response = await self.torso_client.send_request_async(position=torso_height, vel=0.8)
            if not call_response.success:
                response.success = False
                response.status = f"Torso command failed with code {call_response.status}."
                return response
            
            # Move the arms to the desired position
            call_response = await self.right_arm_client.send_request_async(x=x, y=y_right, z=z, vel=vel, grasp_pose=True)
            if not call_response.success:
                response.success = False
                response.status = f"Right arm command failed with code {call_response.status}."
                return response
            
            call_response  = await self.left_arm_client.send_request_async(x=x, y=y_left, z=z, vel=vel, grasp_pose=True)
            if not call_response.success:
                response.success = False
                response.status = f"Left arm command failed with code {call_response.status}."
                return response
            

            # Open the grippers
            call_response  = await self.right_gripper_client.send_request_async(close = False)
            if not call_response.success:
                response.success = False
                response.status = f"Right gripper command failed with code {call_response.status}."
                return response
            

            call_response = await self.left_gripper_client.send_request_async(close = False)
            if not call_response.success:
                response.success = False
                response.status = f"Left gripper command failed with code {call_response.status}."
                return response


            response.success = True
            response.status = "Tiago Dual Grasping Test Node is now enabled."
        else:
            response.success = False
            response.status = "Tiago Dual Grasping Test Node is already enabled."
        return response
    
    def disable_callback(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Disable command received.")
        if self.enabled:
            self.enabled = False
            response.success = True
            response.status = "Tiago Dual Grasping Test Node is now disabled."
            return response
        else:
            response.success = False
            response.status = "Tiago Dual Grasping Test Node is already disabled."
            return response
    
    async def pick_callback(self, request: ArmControl.Request, response: ArmControl.Response):
        self.get_logger().info("Pick command received.")
        if self.enabled:
            x = request.x
            y = request.y
            z = request.z
            obj_to_pick = request.named_pose

            if obj_to_pick:
                # Obtain the transform between the object and the base frame
                transform, error_msg = self.obtain_transform(target_frame="base_footprint", source_frame=obj_to_pick)
                if transform is None:
                    response.success = False
                    response.status = f"Transform error: {error_msg}"
                    return response
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z
                self.get_logger().info(f"Object {obj_to_pick} detected at x: {x}, y: {y}, z: {z}")

            # Constraint the z coordinate to a safe value
            z = max(z, 0.83) 


            await self.pick_place_sequence(x, y, z, response, action="pick")
            if response.success:
                self.get_logger().info("Pick sequence completed successfully.")
                return response
            else:
                self.get_logger().error(f"Pick sequence failed: {response.status}")
                return response
        else:
            self.get_logger().error("Tiago Dual Grasping Node is not enabled.")
            response.success = False
            response.status = "Tiago Dual Grasping Node is not enabled."
            return response

    async def place_callback(self, request: ArmControl.Request, response: ArmControl.Response):
        self.get_logger().info("Place command received.")
        if self.enabled:
            x = request.x
            y = request.y
            z = request.z
            obj_to_pick = request.named_pose

            if obj_to_pick:
                # Obtain the transform between the object and the base frame
                transform, error_msg = self.obtain_transform(target_frame="base_footprint", source_frame=obj_to_pick)
                if transform is None:
                    response.success = False
                    response.status = f"Transform error: {error_msg}"
                    return response
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z
                self.get_logger().info(f"Object {obj_to_pick} detected at x: {x}, y: {y}, z: {z}")

            # Constraint the z coordinate to a safe value
            z = max(z, 0.87) 


            await self.pick_place_sequence(x, y, z, response, action="place")
            if response.success:
                self.get_logger().info("Pick sequence completed successfully.")
                return response
            else:
                self.get_logger().error(f"Pick sequence failed: {response.status}")
                return response
        else:
            self.get_logger().error("Tiago Dual Grasping Node is not enabled.")
            response.success = False
            response.status = "Tiago Dual Grasping Node is not enabled."
            return response
        
    async def pick_place_sequence(self, x, y, z, response, action=""):
        if y < 0:
            arm=self.right_arm_client
            gripper = self.right_gripper_client
            y_sign = -1
            side = "Right"
        else:
            arm=self.left_arm_client
            gripper = self.left_gripper_client
            y_sign = 1
            side = "Left"

        if action == "pick":
            gripper_action = True
        elif action == "place":
            gripper_action = False
        else:
            response.success = False
            response.status = "Invalid action. Use 'pick' or 'place'."
            return

        # Move to approach position
        call_response = await arm.send_request_async(x=x, y=y, z=self.approach_z, vel=self.fast_vel, grasp_pose=True)
        if not call_response.success:
            response.success = False
            response.status = f"{side} arm command failed with code {call_response.status}."
            return
        
        # Move to the object position
        call_response = await arm.send_request_async(x=x, y=y, z=z, vel=self.slow_vel, grasp_pose=True, cartesian=True)
        if not call_response.success:
            response.success = False
            response.status = f"{side} arm command failed with code {call_response.status}."
            return

        # Actuate the gripper
        call_response  = await gripper.send_request_async(close = gripper_action)
        # TODO: Add improved gripper object detection and handling
        # if not call_response.success:
        #     response.success = False
        #     response.status = f"{side} gripper command failed."
        #     return
        
        # Move to the approach position
        call_response = await arm.send_request_async(x=x, y=y, z=self.approach_z, vel=self.slow_vel, grasp_pose=True, cartesian=True)
        if not call_response.success:
            response.success = False
            response.status = f"{side} arm command failed with code {call_response.status}."
            return
        
        # Move to the home position
        call_response = await arm.send_request_async(x=self.home_x, y=y_sign*self.home_y, z=self.home_z, vel=self.fast_vel, grasp_pose=True)
        if not call_response.success:
            response.success = False
            response.status = f"{side} arm command failed with code {call_response.status}."
            return
        
        response.success = True
        

    def obtain_transform(self, target_frame, source_frame):
        """
        Obtain the transform between two frames.

        :param target_frame: The target frame to transform to.
        :type target_frame: str
        :param source_frame: The source frame to transform from.
        :type source_frame: str
        :return: The transform between the two frames.
        :rtype: TransformStamped
        """
        try: 
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform, None
        except Exception as e:
            self.get_logger().error(f"Error obtaining transform: {e}")
            return None, e

class ServiceClientAsync():
    """
    A generic service client class.
    """
    def __init__(self, node:Node, service_type, service_name, callback_group) -> None:
        """
        Constructor for the ServiceClient class.

        :param node: ROS2 node that will host the service client
        :type node: Node
        :param service_type: Message Interface
        :type service_type: Any ROS2 service interface
        :param service_name: Name of the service
        :type service_name: str
        :param callback_group: Callback group to assign the service client
        :type callback_group: rclpy.callback_groups.*
        """        
        self.node=node
        self.cli = self.node.create_client(service_type, service_name, callback_group=callback_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'Service {service_name} not available, waiting again...')
        self.req = service_type.Request()

    def send_request_async(self, **kwargs) -> Future:
        """
        Send a request to the service using asyncio.

        :param kwargs: Keyword arguments representing the request parameters.
        :type kwargs: dict
        :return: The response from the service.
        :rtype: type
        """        
        for key, value in kwargs.items():
            setattr(self.req, key, value)
        self.future = self.cli.call_async(self.req)
        return self.future
    


def pick_and_place(args=None):
    rclpy.init(args=args)
    node = TiagoDualGraspingTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()