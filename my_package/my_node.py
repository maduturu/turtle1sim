import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtlesimGoal(Node):
    def __init__(self):
        super().__init__('turtlesim_goal')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.move_turtle)

        # Posición deseada
        self.goal_x = 2.0
        self.goal_y = 1.0
        
        # Inicializar posicion actual y obj
        self.pose = None
        self.reached = False

    def pose_callback(self, msg):
        self.pose = msg

    def move_turtle(self):   #Si llega para
        if self.pose is None or self.reached:
            return

        msg = Twist()

        # Calcular distancia al objetivo
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2) # Pitagoras para encontrar la distancia

        # Si estamos cerca, detener la tortuga
        if distance < 0.1:
            msg.linear.x = 0.0 # Para vel linear
            msg.angular.z = 0.0 # Para vel angul
            self.reached = True # Indicar que se ha llegaddo
            self.get_logger().info("¡Objetivo alcanzado!")
        else:
            # Control proporcional para ir hacia el objetivo
            angle_to_goal = math.atan2(dy, dx) # atan2 -> calcula el angulo entre eje X y el camino desde donde viene la tortuga
            angle_diff = angle_to_goal - self.pose.theta # resta los dos angulos para saber la orientacion que debe de coger

            # Normalizar ángulo
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            msg.linear.x = min(1.5, distance) 
            msg.angular.z = 4.0 * angle_diff # Se multiplica por 4 (Kp), numero variable, para girar hacia la distancia menor, en caso de ser angulo 350, que sire -10 y no una vuelta entera

            self.get_logger().info(f"Moviendo a ({self.goal_x}, {self.goal_y})")
            self.get_logger().info(f"Posicion actual ({self.pose.x}, {self.pose.y})") # Printar posicion objetivo y actual
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

