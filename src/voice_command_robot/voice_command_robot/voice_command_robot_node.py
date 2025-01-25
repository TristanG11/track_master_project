import rclpy
from rclpy.node import Node
import threading
import speech_recognition as sr
from geometry_msgs.msg import Twist  # Pour commander les vitesses
import sounddevice
from std_msgs.msg import String
class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('twist_topic', '/cmd_vel'),
                ('cmd_type_topic', '/cmd_type'),
                ('standard_lin_vel', 0.5),
                ('standard_ang_vel',0.5)
            ]
        )

        self.twist_topic = self.get_parameter('twist_topic').value
        self.cmd_type_topic = self.get_parameter('cmd_type_topic').value
        self.standard_lin_vel = self.get_parameter('standard_lin_vel').value
        self.standard_ang_vel = self.get_parameter('standard_ang_vel').value

        self.publisher_ = self.create_publisher(Twist,self.twist_topic, 10)  # Publie les commandes de vitesse
        self.recognizer = sr.Recognizer()
        self.current_command = Twist()  # Commande actuelle (par défaut immobile)

        # Timers ROS2
        self.publisher_timer = self.create_timer(0.1, self.publish_cmd)  # Publication à 10 Hz

        # Lancer un thread séparé pour la reconnaissance vocale
        self.voice_thread = threading.Thread(target=self.process_voice)
        self.voice_thread.daemon = True
        self.voice_thread.start()
        self.cmd_type_topic = self.create_subscription(String, self.cmd_type_topic,self.cmd_type_cb,10)
        self.cmd_type = "app_joystick"

    def response_to_query(self, text):
        response = Twist()
        if "avance" in text.lower():
            response.linear.x = self.standard_lin_vel
        elif "recule" in text.lower():
            response.linear.x = -self.standard_lin_vel
        elif "stop" in text.lower():
            response.linear.x = 0.0
            response.angular.z = 0.0
        elif "droite" in text.lower():
            response.linear.x = 0.0
            response.angular.z = -self.standard_ang_vel
        elif "gauche" in text.lower():
            response.linear.x = 0.0
            response.angular.z = self.standard_ang_vel
        else:
            response = None  # Pas de correspondance
        return response

    def process_voice(self):
        """Thread séparé pour écouter les commandes vocales."""
        while True:
            with sr.Microphone() as source:
                self.recognizer.adjust_for_ambient_noise(source)
                try:
                    print("Écoute en cours...")
                    audio = self.recognizer.listen(source)
                    print("Ecoute finie")
                    query = self.recognizer.recognize_google(audio, language="fr-FR")
                    print(f"Vous avez dit : {query}")

                    # Mettre à jour la commande actuelle
                    new_command = self.response_to_query(query)
                    if new_command:
                        self.current_command = new_command
                except sr.UnknownValueError:
                    print("Je n'ai pas compris. Essayez encore.")
                except sr.RequestError as e:
                    print(f"Erreur avec le service de reconnaissance vocale : {e}")

    def publish_cmd(self):
        if self.cmd_type != "voice_command":
            return 
        """Publie la commande actuelle à 10 Hz."""
        self.publisher_.publish(self.current_command)
        

    def cmd_type_cb(self, msg):
        self.cmd_type = msg.data
        #self.get_logger().info(f"Received cmd_type: {self.cmd_type}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
