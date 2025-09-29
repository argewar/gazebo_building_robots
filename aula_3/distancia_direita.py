#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class DistanciaDireita(Node):
    def __init__(self):
        super().__init__('distancia_direita')
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32, '/distancia_direita', 10)
        self.get_logger().info('Publicando distancia_direita em /distancia_direita ...')

    def cb(self, msg: LaserScan):
        ang = math.radians(-90.0)  # direita = -90° (em rad)
        if not (msg.angle_min <= ang <= msg.angle_max):
            self.get_logger().info('distancia_direita: -90° fora do LIDAR')
            return

        i = int(round((ang - msg.angle_min) / msg.angle_increment))
        i = max(0, min(i, len(msg.ranges) - 1))

        r = msg.ranges[i]
        if math.isfinite(r) and (msg.range_min < r < msg.range_max):
            valor = float(r)
        else:
            # se inválido/inf, trate como "livre"
            valor = float(msg.range_max)

        self.pub.publish(Float32(data=valor))
        self.get_logger().info(f'distancia_direita = {valor:.2f} m')

def main():
    rclpy.init()
    n = DistanciaDireita()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
