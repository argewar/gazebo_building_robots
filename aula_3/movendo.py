import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class MovimentoContinuoNode(Node):
    """
    Um nó que publica continuamente comandos de velocidade para fazer o robô se mover.
    """
    def __init__(self):
        super().__init__('movimento_continuo_node')
        
        # Cria um publisher para o tópico /cmd_vel, usando o tipo de mensagem TwistStamped
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Define a frequência de publicação (10 Hz = 0.1 segundos)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Define as velocidades desejadas
        self.velocidade_linear = 0.2  # metros/segundo (para frente)
        self.velocidade_angular = 0.3  # radianos/segundo (curva para a esquerda)
        
        self.get_logger().info('Nó de movimento contínuo (TwistStamped) iniciado.')
        self.get_logger().info(f'Publicando velocidade linear: {self.velocidade_linear} m/s')
        self.get_logger().info(f'Publicando velocidade angular: {self.velocidade_angular} rad/s')

    def timer_callback(self):
        """
        Esta função é chamada pelo timer na frequência definida.
        """
        # Cria uma nova mensagem do tipo TwistStamped
        msg = TwistStamped()
        
        # Preenche o cabeçalho (Header) da mensagem com o tempo atual e o frame de referência
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Preenche a parte 'twist' da mensagem com as velocidades desejadas
        msg.twist.linear.x = self.velocidade_linear
        msg.twist.angular.z = self.velocidade_angular
        
        # Publica a mensagem no tópico /cmd_vel
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    movimento_continuo_node = MovimentoContinuoNode()

    rclpy.spin(movimento_continuo_node)

    movimento_continuo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()