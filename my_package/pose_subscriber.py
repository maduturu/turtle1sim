import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose  # Mensaje que publica turtlesim

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription  # Evita que Python lo elimine

    def pose_callback(self, msg):
        self.get_logger().info(f'Posición de la tortuga - x: {msg.x}, y: {msg.y}, theta: {msg.theta}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
