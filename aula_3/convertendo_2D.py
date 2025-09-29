import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class Lidar2DConverterNode(Node):
    """
    Um nó que se inscreve no tópico do LIDAR, converte os dados para
    coordenadas Cartesianas 2D e exibe alguns pontos.
    """
    def __init__(self):
        super().__init__('lidar_2d_converter_node')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.get_logger().info("Nó conversor de LIDAR para 2D iniciado. Aguardando dados do tópico /scan...")

    def scan_callback(self, msg):
        """
        Esta função é chamada a cada nova mensagem LaserScan recebida.
        """
        self.get_logger().info("--- Scan LIDAR Recebido (Convertido para Coordenadas 2D) ---")

        # Imprime o número total de pontos no scan
        num_pontos = len(msg.ranges)
        self.get_logger().info(f"Total de pontos no scan: {num_pontos}")

        pontos_cartesianos = []
        for i, distancia in enumerate(msg.ranges):
            # Ignora leituras inválidas (infinitas ou fora do alcance)
            if math.isinf(distancia) or math.isnan(distancia):
                continue

            # Calcula o ângulo para a medição atual
            # ângulo = ângulo_inicial + (índice_da_leitura * incremento_angular)
            angulo = msg.angle_min + i * msg.angle_increment

            # Converte de coordenadas polares (distância, ângulo) para Cartesianas (x, y)
            # x = r * cos(θ)
            # y = r * sin(θ)
            ponto_x = distancia * math.cos(angulo)
            ponto_y = distancia * math.sin(angulo)
            
            pontos_cartesianos.append((ponto_x, ponto_y))

        # Exibe as coordenadas de alguns pontos para verificação
        if pontos_cartesianos:
            # Ponto correspondente à frente do robô (primeira leitura)
            ponto_frente = pontos_cartesianos[0]
            self.get_logger().info(f"  Ponto à Frente:     (x={ponto_frente[0]:.2f}, y={ponto_frente[1]:.2f})")

            # Ponto a 90 graus à esquerda (aproximadamente)
            indice_esquerda = num_pontos // 4
            if indice_esquerda < len(pontos_cartesianos):
                 ponto_esquerda = pontos_cartesianos[indice_esquerda]
                 self.get_logger().info(f"  Ponto à Esquerda:   (x={ponto_esquerda[0]:.2f}, y={ponto_esquerda[1]:.2f})")

            # Ponto atrás do robô (aproximadamente)
            indice_atras = num_pontos // 2
            if indice_atras < len(pontos_cartesianos):
                ponto_atras = pontos_cartesianos[indice_atras]
                self.get_logger().info(f"  Ponto Atrás:        (x={ponto_atras[0]:.2f}, y={ponto_atras[1]:.2f})")
        
        self.get_logger().info("-----------------------------------------------------------\n")


def main(args=None):
    rclpy.init(args=args)
    lidar_2d_converter = Lidar2DConverterNode()
    rclpy.spin(lidar_2d_converter)

    lidar_2d_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()