#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class DistanciaFrente(Node):
    def __init__(self):
        super().__init__('distancia_frente')
        # Assina o LIDAR com QoS de sensor
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        # Publica a distância à frente em /dist_frente (Float32)
        self.pub = self.create_publisher(Float32, '/distancia_frente', 10)
        self.get_logger().info('Lendo /scan e publicando e imprimindo distancia_frente em /dist_frente ...')

    def cb(self, msg: LaserScan):
        ang = 0.0  # frente (0 rad)
        # Verifica se 0 rad está no campo de visão do LIDAR
        if not (msg.angle_min <= ang <= msg.angle_max):
            self.get_logger().info('distancia_frente: ângulo 0° fora do FOV do LIDAR')
            return

        # Converte ângulo em índice do vetor ranges
        i = int(round((ang - msg.angle_min) / msg.angle_increment))
        i = max(0, min(i, len(msg.ranges) - 1))

        # ... dentro de cb(...)
        r = msg.ranges[i]
        if math.isfinite(r) and (msg.range_min < r < msg.range_max):
            valor = float(r)
        else:
            valor = float(msg.range_max)   # publica "livre" em vez de não publicar

        self.pub.publish(Float32(data=valor))


def main():
    rclpy.init()
    n = DistanciaFrente()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
