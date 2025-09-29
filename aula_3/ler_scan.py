import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarSubscriberNode(Node):
    """
    Um nó para se inscrever no tópico do LIDAR e exibir os dados.
    """
    def __init__(self):
        # Inicializa o nó com o nome 'lidar_subscriber_node'
        super().__init__('lidar_subscriber_node')
        
        # Cria um subscriber para o tópico /scan
        # O tipo da mensagem é LaserScan
        # A função 'scan_callback' será chamada a cada nova mensagem
        # O '10' é o tamanho da fila (QoS)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # Apenas para evitar aviso de variável não utilizada

    def scan_callback(self, msg):
        """
        Esta função é chamada toda vez que uma nova mensagem LaserScan é recebida.
        """
        # Limpa o terminal para uma visualização mais limpa (opcional)
        print("\033[H\033[J") 
        
        self.get_logger().info("--- Recebendo Dados do LIDAR ---")

        # Exibe informações do cabeçalho
        self.get_logger().info(f"  Frame ID: {msg.header.frame_id}")
        self.get_logger().info(f"  Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

        # Exibe os parâmetros do scan
        self.get_logger().info(f"  Ângulo Mínimo (rad): {msg.angle_min:.4f} ({math.degrees(msg.angle_min):.2f}°)")
        self.get_logger().info(f"  Ângulo Máximo (rad): {msg.angle_max:.4f} ({math.degrees(msg.angle_max):.2f}°)")
        self.get_logger().info(f"  Incremento Angular (rad): {msg.angle_increment:.4f} ({math.degrees(msg.angle_increment):.2f}°)")
        
        # Exibe os limites de alcance
        self.get_logger().info(f"  Alcance Mínimo (m): {msg.range_min:.2f}")
        self.get_logger().info(f"  Alcance Máximo (m): {msg.range_max:.2f}")

        # Exibe o número total de medições no array 'ranges'
        num_leituras = len(msg.ranges)
        self.get_logger().info(f"  Total de Leituras por Scan: {num_leituras}")

    


def main(args=None):
    # Inicializa a biblioteca rclpy
    rclpy.init(args=args)
    # Cria uma instância do nosso nó
    lidar_subscriber = LidarSubscriberNode()
    # Mantém o nó "vivo" para continuar recebendo mensagens
    rclpy.spin(lidar_subscriber)
    # Destroi o nó quando o programa é fechado
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()