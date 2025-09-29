import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Movendo(Node):
    def __init__(self):
        super().__init__('movendo')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz
        self.vx = 0.1   # m/s
        self.wz = 0.05   # rad/s
        self.get_logger().info('Publicando Twist em /cmd_vel (10 Hz).')

    def _tick(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.angular.z = self.wz
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = Movendo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
