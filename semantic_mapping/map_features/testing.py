import sys

from roomba_msgs.srv import GetPathObstacles
from roomba_msgs.msg import MultifloorPath
from roomba_msgs.msg import MultifloorPoint
from roomba_msgs.msg import MultifloorRectangle
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetPathObstacles, 'path_features')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPathObstacles.Request()

    def send_request(self):
        msg = MultifloorPath()
        point1 = MultifloorPoint()
        point1.point.x = 0.0
        point1.point.y = 0.0
        point1.floor_id.data = "1"
        point2 = MultifloorPoint()
        point2.point.x = 3.0
        point2.point.y = 3.0
        point2.floor_id.data = "1"
        points = [point1, point2]
        msg.points = points

        self.req.path = msg
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()
    print(response.features_in_path)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()