#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int32

class JoyToArduino(Node):
    def __init__(self):
        super().__init__('joy_to_arduino')
        
        # Ladataan parametrit yaml-tiedostosta
        self.declare_parameter('action_button', 13)
        self.declare_parameter('motor_speed_value', 100)

        # Haetaan arvot (esim. action_button = 3)
        self.btn_idx = self.get_parameter('action_button').value
        self.speed_val = self.get_parameter('motor_speed_value').value

        # Julkaisijat
        self.rele1_pub = self.create_publisher(Bool, 'rele1_cmd', 10)
        self.rele2_pub = self.create_publisher(Bool, 'rele2_cmd', 10)
        self.pwm_pub = self.create_publisher(Int32, 'motor_pwm_cmd', 10)

        # Tilaaja ohjaimen raakadatalle (/joy)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.get_logger().info(f"Kuunnellaan /joy topicia. Nappi indeksissä {self.btn_idx} aktivoi järjestelmän.")

    def joy_callback(self, msg):
        # Varmistetaan ensin, että taulukossa on tarpeeksi alkioita, jottei ohjelma kaadu
        if len(msg.buttons) > self.btn_idx:
            
            # msg.buttons[3] lukee taulukon 4. arvon. Jos se on 1, is_pressed = True
            is_pressed = (msg.buttons[self.btn_idx] == 1)

            # Valmistellaan viestit
            r_msg = Bool()
            r_msg.data = is_pressed  # True jos painettu, False jos vapautettu
            
            p_msg = Int32()
            p_msg.data = self.speed_val if is_pressed else 0 # 100 jos painettu, 0 jos vapautettu

            # Julkaistaan viestit ROS2-verkkoon
            self.rele1_pub.publish(r_msg)
            self.rele2_pub.publish(r_msg)
            self.pwm_pub.publish(p_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToArduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()