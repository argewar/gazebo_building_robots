#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# Limiar e ganhos simples
DIST_PARAR    = 0.25   # m -> se frente < isso, gira
DIST_LATERAL  = 0.20   # m -> se lateral < isso, afasta
VEL_MAX       = 0.18   # m/s
GIRO_FORTE    = 0.80   # rad/s
GIRO_LEVE     = 0.50   # rad/s
DIST_LIVRE    = 1.00   # m -> velocidade cheia a partir daqui

class Desvia(Node):
    def __init__(self):
        super().__init__('desvia')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.d_f = float('inf'); self.d_d = float('inf'); self.d_e = float('inf')
        self.sub_f = self.create_subscription(Float32, '/dist_frente',   self.cb_f, 10)
        self.sub_d = self.create_subscription(Float32, '/dist_direita',  self.cb_d, 10)
        self.sub_e = self.create_subscription(Float32, '/dist_esquerda', self.cb_e, 10)
        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info('Desvio (3 entradas) ligado.')

    def cb_f(self, m): self.d_f = float(m.data)
    def cb_d(self, m): self.d_d = float(m.data)
    def cb_e(self, m): self.d_e = float(m.data)

    def _tick(self):
        cmd = Twist()

        # 1) bloqueado à frente -> gira para o lado mais livre
        if self.d_f < DIST_PARAR:
            if self.d_e >= self.d_d:
                cmd.angular.z =  GIRO_FORTE
            else:
                cmd.angular.z = -GIRO_FORTE

        # 2) só direita perto -> afasta (puxa à esquerda)
        elif self.d_d < DIST_LATERAL <= self.d_e:
            cmd.linear.x = VEL_MAX * 0.6
            cmd.angular.z = GIRO_LEVE

        # 3) só esquerda perto -> afasta (puxa à direita)
        elif self.d_e < DIST_LATERAL <= self.d_d:
            cmd.linear.x = VEL_MAX * 0.6
            cmd.angular.z = -GIRO_LEVE

        # 4) livre -> segue reto (escala por “aperto” à frente)
        else:
            escala = (self.d_f - DIST_PARAR) / max(1e-6, (DIST_LIVRE - DIST_PARAR))
            escala = max(0.0, min(1.0, escala))
            cmd.linear.x = VEL_MAX * escala

        self.pub.publish(cmd)

def main():
    rclpy.init(); n = Desvia()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
