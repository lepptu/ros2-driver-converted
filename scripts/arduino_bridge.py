#!/usr/bin/env python3

import rclpy
print("SOLMU KÄYNNISTYY...")
from rclpy.node import Node
from std_msgs.msg import Int32, Bool  # String poistettu, koska lähetämme raakadataa
import serial
import threading

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # --- PARAMETRIT ---
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.get_logger().info(f"Yritetään avata porttia: {port} nopeudella {baud}")
        # --- SARJALIIKENNE ---
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Yhteys Arduinoon avattu portissa {port}")
        except Exception as e:
            self.get_logger().error(f"Yhteyden avaus epäonnistui: {e}")
            raise SystemExit

        # --- MUUTTUJAT ---
        self.rele1_state = 0
        self.rele2_state = 0
        self.pwm_state = 0

        # --- TILAUKSET (Ohjaus käskyjä varten) ---
        self.create_subscription(Bool, 'rele1_cmd', self.rele1_callback, 10)
        self.create_subscription(Bool, 'rele2_cmd', self.rele2_callback, 10)
        self.create_subscription(Int32, 'motor_pwm_cmd', self.pwm_callback, 10)

        # --- JULKAISIJAT (Status-tietoa varten) ---
        # Nyt jokaisella on oma topic
        self.rele1_pub = self.create_publisher(Bool, 'rele1_status', 10)
        self.rele2_pub = self.create_publisher(Bool, 'rele2_status', 10)
        self.pwm_pub = self.create_publisher(Int32, 'motor_pwm_status', 10)

        # --- AJASTIN JA LUKUSÄIE ---
        self.timer = self.create_timer(0.1, self.send_to_arduino)
        self.read_thread = threading.Thread(target=self.read_from_arduino, daemon=True)
        self.read_thread.start()

    # Callbackit päivittävät lähetettävän tiedon
    def rele1_callback(self, msg): self.rele1_state = 1 if msg.data else 0
    def rele2_callback(self, msg): self.rele2_state = 1 if msg.data else 0
    def pwm_callback(self, msg): self.pwm_state = msg.data

    def send_to_arduino(self):
        """Lähettää ohjauskomennot Arduinolle muodossa r1,r2,pwm\n"""
        cmd = f"{self.rele1_state},{self.rele2_state},{self.pwm_state}\n"
        self.ser.write(cmd.encode('utf-8'))

    def read_from_arduino(self):
        """Lukee Arduinon vastauksen ja pilkkoo sen eri topiceihin"""
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        # Pilkotaan merkkijono pilkun kohdalta: "1,0,150" -> ["1", "0", "150"]
                        parts = line.split(',')
                        
                        if len(parts) == 3:
                            # 1. Rele 1 status
                            r1_msg = Bool()
                            r1_msg.data = bool(int(parts[0]))
                            self.rele1_pub.publish(r1_msg)

                            # 2. Rele 2 status
                            r2_msg = Bool()
                            r2_msg.data = bool(int(parts[1]))
                            self.rele2_pub.publish(r2_msg)

                            # 3. PWM status
                            pwm_msg = Int32()
                            pwm_msg.data = int(parts[2])
                            self.pwm_pub.publish(pwm_msg)
                            
                except Exception as e:
                    self.get_logger().warn(f"Virhe datan lukemisessa: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()