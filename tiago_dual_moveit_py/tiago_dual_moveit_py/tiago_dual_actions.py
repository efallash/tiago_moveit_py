import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


from tiago_dual_moveit_py.service_client import ServiceClientAsync
from tiago_dual_moveit_py.tiago_dual_moveit_py import TiagoDualPy
from tf2_ros import Buffer, TransformListener
from tiago_moveit_py_interfaces.srv import ArmControl, GripperControl, PickPlaceAction, TorsoControl, Trigger
from tiago_moveit_py_interfaces.msg import GripperState

class TiagoDualActions(Node):
    def __init__(self):
        super().__init__('tiago_dual_grasping_test')
        self.enabled = False

        #Default positions:
        self.approach_z = 1.05
        self.home_z = 0.95
        self.home_x = 0.2
        self.home_y = 0.3
        self.slow_vel = 0.2
        self.fast_vel = 0.5
        self.torso_position = 0.34

        self.client_cb = MutuallyExclusiveCallbackGroup()
        self.server_cb = MutuallyExclusiveCallbackGroup()
        self.topics_cb = MutuallyExclusiveCallbackGroup()

        self.right_arm_client = ServiceClientAsync(self, ArmControl, "tiago_dual/right_arm_command", self.client_cb)
        self.left_arm_client = ServiceClientAsync(self, ArmControl, "tiago_dual/left_arm_command", self.client_cb)
        self.arm_clients = {
            'right': self.right_arm_client,
            'left': self.left_arm_client
        }
        self.right_gripper_client = ServiceClientAsync(self, GripperControl, "tiago_dual/right_gripper_command", self.client_cb)
        self.left_gripper_client = ServiceClientAsync(self, GripperControl, "tiago_dual/left_gripper_command", self.client_cb)
        self.gripper_clients = {
            'right': self.right_gripper_client,
            'left': self.left_gripper_client
        }
        self.torso_client = ServiceClientAsync(self, TorsoControl, "tiago_dual/torso_command", self.client_cb)

        self.startup_srv = self.create_service(Trigger, "tiago_dual/enable_grasping", self.startup_callback, callback_group=self.server_cb)
        self.disable_srv = self.create_service(Trigger, "tiago_dual/disable_grasping", self.disable_callback, callback_group=self.server_cb)
        self.pick_srv = self.create_service(PickPlaceAction, "tiago_dual/pick", self.pick_callback, callback_group=self.server_cb)
        self.place_srv = self.create_service(PickPlaceAction, "tiago_dual/place", self.place_callback, callback_group=self.server_cb)
        self.change_hands_srv = self.create_service(Trigger, "tiago_dual/change_hands", self.change_hands_callback, callback_group=self.server_cb)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.gripper_state_subscriber = self.create_subscription(
            GripperState, "tiago_dual/gripper_state", self.gripper_state_callback, qos_profile=qos, callback_group=self.topics_cb
        )
        self.grasped_right = False
        self.grasped_left = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Tiago Dual Grasping Test Node Initialized.")     

    def gripper_state_callback(self, msg: GripperState):
        """
        Callback to handle gripper state updates.
        """
        self.grasped_right = msg.right.opening > 0.001 and msg.right.current > 0.1
        self.grasped_left = msg.left.opening > 0.001 and msg.left.current > 0.1
        self.get_logger().debug(f"Gripper state updated: Right grasped: {self.grasped_right}, Left grasped: {self.grasped_left}") 

    async def startup_callback(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Startup command received.")
        if not self.enabled:
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

            self.enabled = True
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
    
    async def pick_callback(self, request: PickPlaceAction.Request, response: PickPlaceAction.Response):
        self.get_logger().info("Pick command received.")
        if self.enabled:
            x = request.x
            y = request.y
            z = request.z
            obj_to_pick = request.frame_id

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
            z = max(z, 0.81)


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

    async def place_callback(self, request: PickPlaceAction.Request, response: PickPlaceAction.Response):
        self.get_logger().info("Place command received.")
        if self.enabled:
            x = request.x
            y = request.y
            z = request.z
            obj_to_pick = request.frame_id

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
        
    async def change_hands_callback(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Change hands command received.")
        await self.change_hands_sequence(response)
        return response

    async def change_hands_sequence(self, response):
        
        if self.enabled:
            if self.grasped_right or self.grasped_left:
                reciever_arm = 'left' if self.grasped_right else 'right'
                giver_arm = 'right' if reciever_arm == 'left' else 'left'
                self.get_logger().info(f"Changing hands: {giver_arm} arm will give, {reciever_arm} arm will receive.")
                # Move the arms to the pre-give and pre-collect positions
                await self.arm_clients[giver_arm].send_request_async(named_pose='pre-give', vel=self.fast_vel)
                await self.arm_clients[reciever_arm].send_request_async(named_pose='pre-collect', vel=self.fast_vel)
                # Open the gripper of the receiving arm
                await self.gripper_clients[reciever_arm].send_request_async(close=False)
                # Move the giver arm to the give position
                await self.arm_clients[giver_arm].send_request_async(named_pose='give', vel=self.slow_vel)
                # Move the receiver arm to the collect position
                await self.arm_clients[reciever_arm].send_request_async(named_pose='collect', vel=self.slow_vel)
                # Open the gripper of the giving arm
                await self.gripper_clients[giver_arm].send_request_async(close=False)
                # Close the gripper of the receiving arm
                gripper_response = await self.gripper_clients[reciever_arm].send_request_async(close=True)
                # Move the arms back to the pre-give and pre-collect positions
                await self.arm_clients[giver_arm].send_request_async(named_pose='pre-give', vel=self.slow_vel)
                await self.arm_clients[reciever_arm].send_request_async(named_pose='pre-collect', vel=self.slow_vel)

                # Move arms to home position
                self.get_logger().info("Moving arms to home position...")
                await self.arm_clients[giver_arm].send_request_async(x=self.home_x, y=self.home_y if giver_arm == 'left' else -self.home_y, z=self.home_z, vel=self.fast_vel, grasp_pose=True)
                await self.arm_clients[reciever_arm].send_request_async(x=self.home_x, y=self.home_y if reciever_arm == 'left' else -self.home_y, z=self.home_z, vel=self.fast_vel, grasp_pose=True)

                if getattr(self, f'grasped_{reciever_arm}') and gripper_response.success:
                    status = "Hands changed successfully."
                    self.get_logger().info(status)
                    response.success = True
                    response.status = status
                else:
                    await self.gripper_clients[reciever_arm].send_request_async(close=False)  # Ensure the gripper is open
                    status = f"Object transfer failed"
                    self.get_logger().error(status)
                    response.success = False
                    response.status = status


            else:
                self.get_logger().error("No object is grasped by either arm. Cannot change hands.")
                response.success = False
                response.status = "No object is grasped by either arm. Cannot change hands."
                return response
        else:
            self.get_logger().error("Tiago Dual Grasping Node is not enabled.")
            response.success = False
            response.status = "Tiago Dual Grasping Node is not enabled."


        
    async def pick_place_sequence(self, x, y, z, response, action=""):
        success = True
        status = ""

        # Configure the sequence based on the action
        if action == "pick":
            # Determine which arm to use based on the y coordinate
            if y < 0:
                side = "Right"
            else:
                side = "Left"

            gripper_action = True
            if self.grasped_right or self.grasped_left:
                response.success = False
                response.status = f"Arms already grasping an object. Please release it first."
                return
        elif action == "place":
            if (not self.grasped_left) and (not self.grasped_right):
                response.success = False
                response.status = f"No arm is grasping any object. Please pick an object first."
                return
            elif self.grasped_right:
                side = "Right"
            else:
                side = "Left"
            gripper_action = False
        else:
            response.success = False
            response.status = "Invalid action. Use 'pick' or 'place'."
            return
        
        # Determine the arm and gripper to use
        if side == "Right":
            arm=self.right_arm_client
            gripper = self.right_gripper_client
            y_sign = -1
        elif side == "Left":
            arm=self.left_arm_client
            gripper = self.left_gripper_client
            y_sign = 1
        else:
            response.success = False
            response.status = "Invalid side. Use 'Right' or 'Left'."
            return

        # Move to approach position
        call_response = await arm.send_request_async(x=x, y=y, z=self.approach_z, vel=self.fast_vel, grasp_pose=True, cartesian=True)
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
        self.get_logger().info(f"{side} gripper action {action}: call_response: {call_response.success}")
        
        if action=="pick" and not call_response.success:
            status = f"{side} pick action not successful"
            self.get_logger().error(status)
            await gripper.send_request_async(close = False)
            
        
        # Move to the approach position
        call_response = await arm.send_request_async(x=x, y=y, z=self.approach_z, vel=self.slow_vel, grasp_pose=True, cartesian=True)
        if not call_response.success:
            response.success = False
            response.status = f"{side} arm command failed with code {call_response.status}."
            return
        
        # Move to the home position
        call_response = await arm.send_request_async(x=self.home_x, y=y_sign*self.home_y, z=self.home_z, vel=self.fast_vel, grasp_pose=True, cartesian=True)
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

class TiagoDemo(Node):
    def __init__(self):
        super().__init__('tiago_demo')
        # Create separate callback groups for each client and the service
        self.server_cb = MutuallyExclusiveCallbackGroup()
        self.client_cb = MutuallyExclusiveCallbackGroup()


        from tiago_dual_moveit_py.service_client import ServiceClientAsync
        from tiago_moveit_py_interfaces.srv import PickPlaceAction, Trigger

        self.pick_client = ServiceClientAsync(self, PickPlaceAction, "tiago_dual/pick", self.client_cb)
        self.place_client = ServiceClientAsync(self, PickPlaceAction, "tiago_dual/place", self.client_cb)
        self.change_hands_client = ServiceClientAsync(self, Trigger, "tiago_dual/change_hands", self.client_cb)
        self.demo_srv = self.create_service(Trigger, "tiago_dual/demo", self.demo_callback, callback_group=self.server_cb)
        self.get_logger().info("TiagoDemo node initialized.")

    async def demo_callback(self, request, response):
        
        self.get_logger().info("Demo command received.")
        # Pick 
        
        await self.pick_client.send_request_async(frame_id="detection_lettuce_1")
        await self.change_hands_client.send_request_async()
        await self.place_client.send_request_async(frame_id="detection_pocket_1")

        await self.pick_client.send_request_async(frame_id="detection_apple_1")
        await self.change_hands_client.send_request_async()
        await self.place_client.send_request_async(frame_id="detection_basket_1")
    
        self.get_logger().info("Demo sequence completed.")
        response.success = True
        response.status = "Demo sequence completed successfully."
        return response

def pick_and_place(args=None):
    rclpy.init(args=args)
    node = TiagoDualActions()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

def main_demo(args=None):
    rclpy.init(args=args)
    node = TiagoDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()