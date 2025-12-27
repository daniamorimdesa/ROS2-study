import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

"""
O código lê e exibe os dados de odometria do robô TIAgo:

- Subscreve ao tópico /ground_truth_odom (Odometry)
- Extrai informações de posição (x, y) e orientação (quaternion)
- Converte o quaternion para ângulos de Euler (roll, pitch, yaw)
- Imprime continuamente: x, y, yaw
"""


class TiagoClass(Node):

	def __init__(self):
		super().__init__('tiago_node')
		
		# Subscriber
		self.create_subscription(Odometry, '/ground_truth_odom', self.pose_cb, 1)
		
	# função de callback para mensagens de odometria
	def pose_cb(self, msg):
		
		# Extrair posição
		x = msg.pose.pose.position.x # armazenar a posição x
		y = msg.pose.pose.position.y # armazenar a posição y

		# extrair a orientação em quaternion
		qtn = msg.pose.pose.orientation 

		# converter quaternion para lista
		qtn_list = [qtn.x, qtn.y, qtn.z, qtn.w] 

		# converter quaternion para ângulos de Euler
		(roll, pitch, yaw) = euler_from_quaternion(qtn_list) 

		print(x, y, yaw)
		
		
def main(args=None):
    rclpy.init(args=args)
    my_tiago = TiagoClass()
    rclpy.spin(my_tiago)
    my_tiago.destroy_node()
    rclpy.shutdown()
#-------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    main()
		