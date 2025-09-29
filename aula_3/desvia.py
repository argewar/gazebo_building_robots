#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ====== Ajustes rápidos ======
STOP_DIST   = 0.45   # m: se frente < STOP_DIST, para de ir reto e gira
SIDE_DIST   = 0.40   # m: distância mínima lateral (paredes)
FREE_DIST   = 1.00   # m: distância a partir da qual pode ir na velocidade cheia
V_FWD       = 0.18   # m/s: velocidade máxima à frente
W_TURN      = 0.80   # rad/s: giro forte quando bloqueado na frente
W_NUDGE     = 0.50   # rad/s: giro leve para “puxar” da parede lateral
SECTOR_HALF = 30.0   # graus: meia-largura dos setores (+/-)

def angle_diff(a, b):
    """diferença angular (a-b) normalizada para [-pi, pi]"""
    return math.atan2(math.sin(a - b), math.cos(a - b))

def sector_min(msg: LaserScan, center_deg: float, half_deg: float) -> float:
    """Menor distância válida no setor centrado em center_deg (graus), largura 2*half_deg."""
    center = math.radians(center_deg)
    half   = math.radians(half_deg)
    dmin = msg.range_max
    found = False
    ang = msg.angle_min
    for i, r in enumerate(msg.ranges):
        # ângulo do feixe i
        ang = msg.angle_min + i * msg.angle_increment
        if abs(angle_diff(ang, center)) <= half:
            if math.isfinite(r) and (msg.range_min < r < msg.range_max):
                if r < dmin:
                    dmin = r
                found = True
    return dmin if found else msg.range_max

class Desvia(Node):
    def __init__(self):
        super().__init__('desvia_obstaculos')
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_scan = None
        # controla a 10 Hz
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('Desvio reativo ligado (lendo /scan, publicando /cmd_vel).')

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def _tick(self):
        cmd = Twist()

        # se ainda não chegou nenhum scan, fique parado por segurança
        if self.last_scan is None:
            self.pub.publish(cmd)
            return

        msg = self.last_scan

        # distâncias mínimas nos setores (graus):
        d_front = sector_min(msg, 0.0,  SECTOR_HALF)    # -30..+30
        d_right = sector_min(msg, -90.0, SECTOR_HALF)   # -120..-60
        d_left  = sector_min(msg, +90.0, SECTOR_HALF)   # +60..+120

        # 1) bloqueado à frente -> gira pro lado mais livre
        if d_front < STOP_DIST:
            if d_left >= d_right:
                cmd.angular.z =  W_TURN   # gira à esquerda
            else:
                cmd.angular.z = -W_TURN   # gira à direita

        # 2) muito perto da parede à direita -> puxa à esquerda
        elif d_right < SIDE_DIST <= d_left:
            cmd.linear.x = V_FWD * 0.6
            cmd.angular.z = W_NUDGE

        # 3) muito perto da parede à esquerda -> puxa à direita
        elif d_left < SIDE_DIST <= d_right:
            cmd.linear.x = V_FWD * 0.6
            cmd.angular.z = -W_NUDGE

        # 4) livre -> segue reto (com redução se “apertado”)
        else:
            # escala a velocidade de 0 (em STOP_DIST) até 1 (em FREE_DIST)
            scale = (d_front - STOP_DIST) / max(1e-6, (FREE_DIST - STOP_DIST))
            scale = max(0.0, min(1.0, scale))
            cmd.linear.x = V_FWD * scale

        self.pub.publish(cmd)
        # (opcional) para depurar, descomente:
        # self.get_logger().info(f"front={d_front:.2f} right={d_right:.2f} left={d_left:.2f} -> vx={cmd.linear.x:.2f} wz={cmd.angular.z:.2f}")

def main():
    rclpy.init()
    node = Desvia()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
