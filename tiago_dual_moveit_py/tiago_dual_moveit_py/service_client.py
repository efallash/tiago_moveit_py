
from rclpy.node import Node
from rclpy.task import Future

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
        self.service_type = service_type
        self.req = service_type.Request()

    def send_request_async(self, **kwargs) -> Future:
        """
        Send a request to the service using asyncio.

        :param kwargs: Keyword arguments representing the request parameters.
        :type kwargs: dict
        :return: The response from the service.
        :rtype: type
        """
        self.req = self.service_type.Request()   
        for key, value in kwargs.items():
            setattr(self.req, key, value)
        self.future = self.cli.call_async(self.req)
        return self.future
    
