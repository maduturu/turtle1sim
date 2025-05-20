import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn


class SquareDrawer(Node):
    def __init__(self):
        super().__init__('square_drawer')
        self.cmd_pub = self.create_publisher(Twist, '/square_turtle/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/square_turtle/pose', self.pose_callback, 10)
        self.current_pose = None
        self.targets = [(7.5, 3.0), (4.5, 3.0), (4.5, 0.0), (7.5, 0.0)]
        self.target_index = 0
        self.reached_target = False

        self.spawn_turtle()

    def spawn_turtle(self):
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio de spawn...')

        req = Spawn.Request()
        req.x = 7.5
        req.y = 0.0
        req.theta = math.pi / 2
        req.name = 'square_turtle'

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Tortuga creada en (7.5, 0.0)')

        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None or self.target_index >= len(self.targets):
            return

        target_x, target_y = self.targets[self.target_index]
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.hypot(dx, dy)

        # Umbral para decir que ha llegado al punto
        if distance < 0.1:
            self.get_logger().info(f'Lado {self.target_index + 1} completado')
            self.target_index += 1
            self.cmd_pub.publish(Twist())  # parar
            return

        # Control proporcional simple
        angle_to_target = math.atan2(dy, dx)
        angle_error = angle_to_target - self.current_pose.theta
        # Normalizar ángulo entre -pi y pi
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()
        if abs(angle_error) > 0.1:
            cmd.angular.z = 1.5 * angle_error
        else:
            cmd.linear.x = 2.0 * distance
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = SquareDrawer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
