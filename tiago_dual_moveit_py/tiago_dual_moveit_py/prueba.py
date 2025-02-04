import rclpy
from rclpy.node import Node
import rclpy.time
from tiago_dual_moveit_py.tiago_dual_moveit_py import TiagoDualPy
from geometry_msgs.msg import PoseStamped
import time
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import tf_transformations
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory



class CartesianPathClient(Node):
    def __init__(self):
        super().__init__('cartesian_path_client')
        self.client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.subscriber = self.create_subscription(JointState, "/joint_states", self.read_joints_callback, 1)
        self.robot_state = RobotState()
        # Asegúrate de que el servicio esté disponible antes de enviar la solicitud
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Esperando que el servicio /compute_cartesian_path esté disponible...')

    def send_request(self, waypoints):
        # Crear solicitud
        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'base_link'
        request.start_state = self.robot_state
        request.link_name = "arm_left_tool_link"
        request.waypoints = waypoints
        request.group_name = 'arm_left'  # Cambia esto al grupo correcto
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        # Llamada asíncrona
        self.future = self.client.call_async(request)

    def process_response(self):
        # Procesar respuesta cuando esté lista
        if self.future.done():
            try:
                response = self.future.result()  # Aquí obtienes la respuesta completa del servicio
                self.get_logger().info(f"OPA RACING: {response.solution}")
                return response.fraction, response.solution
            except Exception as e:
                self.get_logger().error(f"Excepción durante la llamada al servicio: {e}")
                return None, None
        else:
            return None, None
    
    def read_joints_callback(self, msg):
        msg = JointState()
        self.robot_state.joint_state = msg
        

def main():

    # ###################################################################
    # # MoveItPy Setup
    # ###################################################################
    rclpy.init()
    tiago_dual=TiagoDualPy()
    node = CartesianPathClient()
    logger=tiago_dual.logger
    # ###########################################################################
    # # Plan 2 - set goal state with RobotState object
    # ###########################################################################
    
    # #TODO
    waypoints = []

    start_pose = Pose()
    start_pose.position.x= 0.5
    start_pose.position.y=0.25
    start_pose.position.z=1.1
    start_pose.orientation.x=0.5
    start_pose.orientation.y=0.5
    start_pose.orientation.z=-0.5
    start_pose.orientation.w=0.5
    waypoints.append(start_pose)
    # Pose final
    pose_target = Pose()
    pose_target.position.x = 0.4
    pose_target.position.y = 0.25
    pose_target.position.z = 1.1
    pose_target.orientation.w = 0.5
    pose_target.orientation.x = 0.5
    pose_target.orientation.y = 0.5
    pose_target.orientation.z = -0.5
    waypoints.append(pose_target)
    
    node.send_request(waypoints)

    # Esperar la respuesta procesándola en el loop
    while rclpy.ok():
        rclpy.spin_once(node)
        fraction, plan = node.process_response()
        if fraction is not None:  # Cuando la respuesta esté lista
            if fraction > 0.9:
                logger.info(f"Path computed successfully: {fraction * 100:.2f}% of waypoints reached.")
            else:
                logger.info(f"Failed to compute a valid path. Fraction: {fraction}")
                logger.info(f"Plan: {plan}")
    
        tiago_dual.action_client = ActionClient(tiago_dual, ExecuteTrajectory, "/execute_trajectory")
        goal = ExecuteTrajectory.Goal()
        logger.info(f"{plan}")
        goal.controller_names = ["arm_left_controller"]
        goal.trajectory = plan
        tiago_dual.action_client.wait_for_server()
        future = tiago_dual.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(tiago_dual, future)
        break
    rclpy.shutdown()


    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    #Right arm pose
    #tiago_dual.move_head(-0.5,0.5)
    #tiago_dual.move_head(0.0,-0.0, sleep_time=1)
    # tiago_dual.move_gripper('left', left_finger=0.044, right_finger=0.0, sleep_time=1)

    # tiago_dual.move_gripper('right', left_finger=0.0, right_finger=0.044, sleep_time=1)
    #tiago_dual.move_head(0.0,0.0)
    # tiago_dual.close_gripper('left', sleep_time=1)
    # tiago_dual.close_gripper('right', sleep_time=1)
    # tiago_dual.open_gripper('left', sleep_time=1)
    # tiago_dual.open_gripper('right', sleep_time=1)
    # tiago_dual.close_gripper('left', sleep_time=2)
    # tiago_dual.open_gripper('right', sleep_time=2)
    #tiago_dual.close_gripper('left', sleep_time=1)

#     #Right arm pose
#     pose_goal = PoseStamped()
#     pose_goal.header.frame_id = "base_footprint"
#     pose_goal.pose.orientation.x = 0.5
#     pose_goal.pose.orientation.y = 0.5
#     pose_goal.pose.orientation.z = -0.5
#     pose_goal.pose.orientation.w = 0.5
#     pose_goal.pose.position.x = 0.5
#     pose_goal.pose.position.y = 0.25
#     pose_goal.pose.position.z = 1.17

# #     # call arm method
# #     logger.info('Moving to goal pose 1 right')
#     tiago_dual.arm_go_to_pose('left', pose_goal, vel_factor=0.2, sleep_time=1)

#         #Right arm pose
#     pose_goal = PoseStamped()
#     pose_goal.header.frame_id = "base_footprint"
#     pose_goal.pose.orientation.x = 0.5
#     pose_goal.pose.orientation.y = 0.5
#     pose_goal.pose.orientation.z = -0.5
#     pose_goal.pose.orientation.w = 0.5
#     pose_goal.pose.position.x = 0.5
#     pose_goal.pose.position.y = -0.25
#     pose_goal.pose.position.z = 1.17

# #     # call arm method
# #     logger.info('Moving to goal pose 1 right')
#     tiago_dual.arm_go_to_pose('right', pose_goal, vel_factor=0.2, sleep_time=1)


#     pose_goal = PoseStamped()
#     pose_goal.header.frame_id = "base_footprint"
#     pose_goal.pose.orientation.x = 0.0
#     pose_goal.pose.orientation.y = 0.0
#     pose_goal.pose.orientation.z = -0.707
#     pose_goal.pose.orientation.w = 0.707
#     pose_goal.pose.position.x = 0.5
#     pose_goal.pose.position.y = 0.25
#     pose_goal.pose.position.z = 0.75

#     # call arm method
#     logger.info('Moving to goal pose 1 left')
#     tiago_dual.arm_go_to_pose('left', pose_goal, vel_factor=0.2, sleep_time=1)


#    #Right arm pose
#     pose_goal = PoseStamped()
#     pose_goal.header.frame_id = "base_footprint"
#     pose_goal.pose.orientation.x = 0.5
#     pose_goal.pose.orientation.y = 0.5
#     pose_goal.pose.orientation.z = -0.5
#     pose_goal.pose.orientation.w = -0.5
#     pose_goal.pose.position.x = 0.5
#     pose_goal.pose.position.y = -0.20
#     pose_goal.pose.position.z = 0.75

#     # call arm method
#     logger.info('Moving to goal pose 1 right')
#     tiago_dual.arm_go_to_pose('right', pose_goal, vel_factor=0.2, sleep_time=1)

#     pose_goal = PoseStamped()
#     pose_goal.header.frame_id = "base_footprint"
#     pose_goal.pose.orientation.x = 0.0
#     pose_goal.pose.orientation.y = 0.0
#     pose_goal.pose.orientation.z = -0.707
#     pose_goal.pose.orientation.w = 0.707
#     pose_goal.pose.position.x = 0.5
#     pose_goal.pose.position.y = 0.20
#     pose_goal.pose.position.z = 0.75

#     # call arm method
#     logger.info('Moving to goal pose 1 left')
#     tiago_dual.arm_go_to_pose('left', pose_goal, vel_factor=0.2, sleep_time=1)

    # tiago_dual.open_gripper("left", 0.1, 1)
    # tiago_dual.open_gripper("right", 0.1, 1)
    # tiago_dual.close_gripper("left", 0.1, 1)

    # tiago_actions.change_hands(giver_arm = 'left', receiver_arm = 'right')

    

    # tiago_dual.arm_go_to_named_pose(arm='right', pose_name='give', vel_factor=0.2, sleep_time=1)
    # tiago_dual.arm_go_to_named_pose(arm='left', pose_name='collect', vel_factor=0.2, sleep_time=1)

if __name__ == "__main__":
    main()