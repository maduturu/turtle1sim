import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen
import math

class SimpleTurtleLine(Node): # Per dibuixar dos linies verticals, representant les linies de moviment del meteorito
    def __init__(self):
        super().__init__('simple_turtle_line')
        self.turtles = {
            'turtle2': (5.0, 9.5), # Posicio inicial de les tortugues
            'turtle3': (6.0, 9.5)
        }
        self.pubs = {}
        self.spawn_turtles()

    def spawn_turtles(self):
        spawn_client = self.create_client(Spawn, 'spawn') # Spawnear tortugues
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            pass

        for name, (x, y) in self.turtles.items():
            req = Spawn.Request()
            req.x = x
            req.y = y
            req.theta = math.pi / 2 # Apuntar cap a dalt 90 graus
            req.name = name
            future = spawn_client.call_async(req) # Solicitud al servei de forma asincrona
            rclpy.spin_until_future_complete(self, future)
            self.pubs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.set_pen_yellow(name)

        self.draw_lines()

    def set_pen_yellow(self, name): # Set pen per pintar les linies de groc
        client = self.create_client(SetPen, f'/{name}/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            pass

        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 0
        req.width = 2
        req.off = 0
        client.call_async(req) # Solicitud de forma asincrona, es a dir no bloqueja el codi

    def draw_lines(self):
        twist = Twist()
        twist.linear.x = 2.0

        for _ in range(50): # Durant 50 iteracions, aprox 5seg
            for pub in self.pubs.values():
                pub.publish(twist)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        for pub in self.pubs.values(): # Publica mensaje vacio per parar les tortgues 
            pub.publish(Twist())


def main():
    rclpy.init()
    node = SimpleTurtleLine()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
