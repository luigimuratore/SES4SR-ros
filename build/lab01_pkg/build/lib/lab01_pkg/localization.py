import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose 
# NON abbiamo bisogno di importare tf_transformations

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        
        # Inizializzazione della posizione (parte dall'origine)
        self.x = 0.0
        self.y = 0.0
        
        # Periodo di campionamento (Delta T) in secondi: 1 s
        self.DT = 1.0 
        
        # 1. Creazione del Publisher per la posizione
        self.publisher_pose = self.create_publisher(Pose, '/pose', 10)
        
        # 2. Creazione della Subscription per la velocità
        self.subscription = self.create_subscription(Twist,'/cmd_vel', self.listener_callback,10)
        self.get_logger().info("Localization node started. Initial position: (0.0, 0.0)")
        

    def listener_callback(self, msg: Twist):
        """
        Funzione di callback chiamata ogni volta che arriva un messaggio Twist.
        """
        # Estrazione delle velocità lineari
        v_x = msg.linear.x
        v_y = msg.linear.y
        
        # --- Stima della Posizione (Integrazione) ---
        # Posizione_nuova = Posizione_vecchia + Velocità * DT
        self.x += v_x * self.DT
        self.y += v_y * self.DT
        
        # --- Pubblicazione della Pose ---
        pose_msg = Pose()
        
        # Posizione (Punto)
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = 0.0

        # Orientamento (Quaternione) - Impostato manualmente
        # Poiché non c'è rotazione, usiamo il Quaternione di identità: (x=0, y=0, z=0, w=1)
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0 # Componente Reale (l'unica non zero)

        self.publisher_pose.publish(pose_msg)

        # --- Stampa con il logger ---
        self.get_logger().info(
            f'Received: vx={v_x:.1f}, vy={v_y:.1f}. '
            f'New Pose published: X={self.x:.2f}, Y={self.y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    localization_node = Localization()

    rclpy.spin(localization_node)

    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()