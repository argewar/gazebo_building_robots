
import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

SETA_MEIA = 30.0  # graus (meia largura do setor)

def dif_angular(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))

class DistFrente(Node):
    def __init__(self):
        super().__init__('dist_frente')
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32, '/dist_frente', 10)
        self.get_logger().info('Publicando /dist_frente (setor 0° ±30°)...')

    def cb(self, scan: LaserScan):
        centro = math.radians(0.0)
        half   = math.radians(SETA_MEIA)
        dmin = scan.range_max
        achou = False
        for i, r in enumerate(scan.ranges):
            ang = scan.angle_min + i*scan.angle_increment
            if abs(dif_angular(ang, centro)) <= half:
                if math.isfinite(r) and scan.range_min < r < scan.range_max:
                    dmin = min(dmin, r); achou = True
        self.pub.publish(Float32(data=dmin if achou else float(scan.range_max)))

def main():
    rclpy.init(); 
    n = DistFrente()
    rclpy.spin(n)
    n.destroy_node(); 
    rclpy.shutdown()

if __name__ == '__main__': main()
