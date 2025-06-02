import rclpy
import numpy as np
from rclpy.node import Node
from collections import defaultdict, deque

from pal_statistics_msgs.msg import Statistics
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from tiago_moveit_py_interfaces.srv import GetActuator  # Use as a generic string request/response
from tiago_moveit_py_interfaces.msg import GripperState  # Custom message for gripper state


class TiagoDualControllerMonitor(Node):
    def __init__(self, name='tiago_dual_controller_monitor'):
        super().__init__(name)
        self.declare_parameter('queue_size', 3)
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value

        # Nested dict: {actuator: {characteristic: deque}}
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.actuator_queues = defaultdict(lambda: defaultdict(lambda: deque(maxlen=self.queue_size)))

        self.subscriber = self.create_subscription(
            Statistics, "actuators_statistics/full", self.actuator_states_callback, qos_profile=qos
        )

        self.srv = self.create_service(
            GetActuator,
            'tiago_dual/get_actuator_characteristic',
            self.get_average_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.gripper_state_publisher = self.create_publisher(GripperState, 'tiago_dual/gripper_state', qos_profile=qos)

        self.get_logger().info("Tiago Dual Controller Monitor is now active.")

    def actuator_states_callback(self, msg: Statistics):
        # Parse only statistics with names starting with "actuators."
        for stat in msg.statistics:
            name = stat.name
            value = stat.value
            if not name.startswith("actuators."):
                continue
            # name: actuators.arm_left_1_actuator.mode_state
            parts = name.split('.')
            if len(parts) < 3:
                continue
            actuator = parts[1]
            characteristic = '.'.join(parts[2:])
            # Save value in queue
            try:
                v = float(value)
                if np.isnan(v):
                    continue
            except Exception:
                continue
            self.actuator_queues[actuator][characteristic].append(v)

        # Publish gripper state if actuator is gripper
        msg = self.get_gripper_state()
        self.gripper_state_publisher.publish(msg)
    
    def get_gripper_state(self):
        gripper_state = GripperState()
        gripper_state.right.opening = self.get_opening_position('right')
        gripper_state.left.opening = self.get_opening_position('left')
        gripper_state.right.current = self.get_average_current('right')
        gripper_state.left.current = self.get_average_current('left')
        return gripper_state


    def get_opening_position(self, arm):
        name_right = f"gripper_{arm}_right_finger_actuator"
        name_left  = f"gripper_{arm}_left_finger_actuator"
        characteristic = "position_state"
        queue_right = self.actuator_queues.get(name_right, {}).get(characteristic, None)
        queue_left  = self.actuator_queues.get(name_left, {}).get(characteristic, None)
        if queue_right is None or queue_left is None or len(queue_right) == 0 or len(queue_left) == 0:
            self.get_logger().warn(f"No data available for {name_right} or {name_left} in characteristic {characteristic}. Returning NaN.")
            return np.nan
        avg_right = float(np.mean(queue_right))
        avg_left = float(np.mean(queue_left))
        opening = avg_right + avg_left
        return opening
    
    def get_average_current(self, arm):
        name_right = f"gripper_{arm}_right_finger_actuator"
        name_left  = f"gripper_{arm}_left_finger_actuator"
        characteristic = "current_state"
        queue_right = self.actuator_queues.get(name_right, {}).get(characteristic, None)
        queue_left  = self.actuator_queues.get(name_left, {}).get(characteristic, None)
        if queue_right is None or queue_left is None or len(queue_right) == 0 or len(queue_left) == 0:
            self.get_logger().warn(f"No data available for {name_right} or {name_left} in characteristic {characteristic}. Returning NaN.")
            return np.nan
        avg_right = abs(float(np.mean(queue_right)))
        avg_left = abs(float(np.mean(queue_left)))
        current = (avg_right + avg_left)/2
        return current
        

    def get_average_callback(self, request, response):
        self.get_logger().info(f"Get average for actuator: {request.name}, characteristic: {request.characteristic}")
        self.get_logger().debug(f"DEBUG: Current queues: {self.actuator_queues}")
        actuator = request.name
        characteristic = request.characteristic
        queue = self.actuator_queues.get(actuator, {}).get(characteristic, None)
        if queue is None or len(queue) == 0:
            response.value = np.nan # TODO: HANDLE CASE WHERE NO DATA IS AVAILABLE
            return response
        avg = float(np.mean(queue))
        response.value = avg
        self.get_logger().info(f"Average for {actuator}.{characteristic} is {avg}")
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = TiagoDualControllerMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()