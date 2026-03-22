#!/usr/bin/env python3

import rclpy
print("SOLMU KÄYNNISTYY...")
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32  # String poistettu, koska lähetämme raakadataa
import serial
import threading
import time

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
        #self.rele1_state = 0
        #self.rele2_state = 0
        #self.pwm_state = 0
        self.eStop_state = 0
        self.mowMotorALM_state = 0
        self.bumperFront_state = 0
        self.mowMotorEN_State = 0
        self.mowMotorRpmSet_state = 0
        self.mowMotorRpmFB_state = 0
        self.mowMotorCur_state = 0.0
        self.hoverBtnR1_state = 0
        self.varaReleR2_state = 0

        # --- TILAUKSET (Ohjaus käskyjä varten) ---
        self.create_subscription(Bool, 'mowMotorEN_cmd', self.mowMotorEN_callback, 10)
        self.create_subscription(Int32, 'mowMotorRPM_set_cmd', self.mowMotorRPM_set_callback, 10)
        self.create_subscription(Bool, 'hoverBtnR1_cmd', self.hoverBtnR1_callback, 10)
        self.create_subscription(Bool, 'varaReleR2_cmd', self.varaReleR2_callback, 10)
        #self.create_subscription(Int32, 'motor_pwm_cmd', self.pwm_callback, 10)
        #self.create_subscription(Bool, 'rele1_cmd', self.rele1_callback, 10)
        #self.create_subscription(Bool, 'rele2_cmd', self.rele2_callback, 10)
        
        

        # --- JULKAISIJAT (Status-tietoa varten) ---
        # Nyt jokaisella on oma topic
        #self.hoverBtnR1_pub = self.create_publisher(Bool, 'hoverBtnR1_status', 10)
        #self.rele2_pub = self.create_publisher(Bool, 'rele2_status', 10)
        #self.pwm_pub = self.create_publisher(Int32, 'motor_pwm_status', 10)
        self.eStop_pub = self.create_publisher(Bool, 'eStop_status', 10)
        self.mowMotorALM_pub = self.create_publisher(Bool, 'mowMotorALM_status', 10)
        self.bumperFront_pub = self.create_publisher(Bool, 'bumperFront_status', 10)
        self.mowMotorEN_pub = self.create_publisher(Bool, 'mowMotorEN_status', 10)
        self.mowMotorRPM_set_pub = self.create_publisher(Int32, 'mowMotorRPM_set_status', 10)
        self.mowMotorRPM_FB_pub = self.create_publisher(Int32, 'mowMotorRPM_FB_status', 10)
        self.mowMotorCur_pub = self.create_publisher(Float32, 'mowMotorCur_status', 10)
        self.hoverBtnR1_pub = self.create_publisher(Bool, 'hoverBtnR1_status', 10)
        self.varaReleR2_pub = self.create_publisher(Bool, 'varaReleR2_status', 10)

        # --- AJASTIN JA LUKUSÄIE ---
        self.timer = self.create_timer(0.1, self.send_to_arduino)
        self.read_thread = threading.Thread(target=self.read_from_arduino, daemon=True)
        self.read_thread.start()

    # Callbackit päivittävät lähetettävän tiedon
    #def hoverBtnR1_callback(self, msg): self.hoverBtnR1_state = 1 if msg.data else 0
    #def varaReleR2_callback(self, msg): self.varaReleR2_state = 1 if msg.data else 0
    #def pwm_callback(self, msg): self.pwm_state = msg.data
    def mowMotorEN_callback(self, msg): self.mowMotorEN_State = 1 if msg.data else 0
    def mowMotorRPM_set_callback(self, msg): self.mowMotorRpmSet_state = msg.data
    def hoverBtnR1_callback(self, msg): self.hoverBtnR1_state = 1 if msg.data else 0
    def varaReleR2_callback(self, msg): self.varaReleR2_state = 1 if msg.data else 0

    def send_to_arduino(self):
        """Lähettää ohjauskomennot Arduinolle muodossa r1,r2,pwm\n"""
        #cmd = f"{self.rele1_state},{self.rele2_state},{self.pwm_state}\n"
        cmd = f"{self.mowMotorEN_State},{self.mowMotorRpmSet_state},{self.hoverBtnR1_state},{self.varaReleR2_state}\n"
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
                        
                        if len(parts) == 9:
                            # 0. eStop status
                            eStop_msg = Bool()
                            eStop_msg.data = bool(int(parts[0]))
                            self.eStop_pub.publish(eStop_msg)

                            # 1. Leikkuumoottori alarm status
                            mowMotorALM_msg = Bool()
                            mowMotorALM_msg.data = bool(int(parts[1]))
                            self.mowMotorALM_pub.publish(mowMotorALM_msg)

                            # 2. Bumper front status
                            bumperFront_msg = Bool()
                            bumperFront_msg.data = bool(int(parts[2]))
                            self.bumperFront_pub.publish(bumperFront_msg)

                            # 3. Leikkuumoottori Enable status
                            mowMotorEN_msg = Bool()
                            mowMotorEN_msg.data = bool(int(parts[3]))
                            self.mowMotorEN_pub.publish(mowMotorEN_msg)

                            # 4. Leikkuumoottori RPM set status
                            mowMotorRPM_set_msg = Int32()
                            mowMotorRPM_set_msg.data = int(parts[4])
                            self.mowMotorRPM_set_pub.publish(mowMotorRPM_set_msg)

                            # 5. Leikkuumoottori RPM FB status
                            mowMotorRPM_FB_msg = Int32()
                            mowMotorRPM_FB_msg.data = int(parts[5])
                            self.mowMotorRPM_FB_pub.publish(mowMotorRPM_FB_msg)

                            # 6. Leikkuumoottori Virta status
                            mowMotorCur_msg = Float32()
                            mowMotorCur_msg.data = float(parts[6])
                            self.mowMotorCur_pub.publish(mowMotorCur_msg)

                            # 7. Hovermower controllerin nappi ON/OFF status
                            hoverBtnR1_msg = Bool()
                            hoverBtnR1_msg.data = bool(int(parts[7]))
                            self.hoverBtnR1_pub.publish(hoverBtnR1_msg)

                            # 8. Varareleen status
                            varaReleR2_msg = Bool()
                            varaReleR2_msg.data = bool(int(parts[8]))
                            self.varaReleR2_pub.publish(varaReleR2_msg)

                except Exception as e:
                    self.get_logger().warn(f"Virhe datan lukemisessa: {e}")
            else:
                # <--- TÄMÄ ON SE TAIKURI, JOKA VAPAUTTAA PROSESSORIN! --->
                # Jos dataa ei ole odottamassa, nukutaan 10 millisekuntia 
                # ennen kuin kysytään uudestaan. Tämä on tarpeeksi nopea reagointiaika 
                # puskurille, mutta antaa prosessorin levätä.
                time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()