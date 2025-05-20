import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.srv import SetPen

class SquareDrawer(Node):
    def __init__(self):
        super().__init__('square_drawer')
        self.spawn_turtle()

    def spawn_turtle(self):
        client = self.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            pass

        req = Spawn.Request()
        req.x = 7.5
        req.y = 0.0
        req.theta = math.pi / 2  # hacia arriba
        req.name = 'square_turtle'

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Tortuga creada en (4.5, 0)')

        self.cmd_pub = self.create_publisher(Twist, '/square_turtle/cmd_vel', 10)
        self.draw_square()

    def draw_square(self):
        move_cmd = Twist()
        move_cmd.linear.x = 2.0  # avanzar

        turn_cmd = Twist()
        turn_cmd.angular.z = math.pi / 2  # girar 90 grados

        for _ in range(4):
            # avanzar
            for _ in range(20):
                self.cmd_pub.publish(move_cmd)
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            # parar
            self.cmd_pub.publish(Twist())
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
            # girar
            for _ in range(10):
                self.cmd_pub.publish(turn_cmd)
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
            self.cmd_pub.publish(Twist())
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))


def main():
    rclpy.init()
    node = SquareDrawer()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
