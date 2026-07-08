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
        self.lidarPWR_state = 0

        # --- LEIKKUUMOOTTORIN WATCHDOG ---
        # Komentajat (MowMotorController, joy_to_arduino) julkaisevat
        # mowMotorEN_cmd:tä 2 Hz keep-alivena. Jos komennot lakkaavat
        # (mission-node kaatui/kill -9, executor jumissa, WiFi poikki),
        # terä sammutetaan sen sijaan että viimeistä enable=1-tilaa
        # toistettaisiin Arduinolle ikuisesti. 1.5 s = 3 väliin jäänyttä
        # keep-alivea.
        self.declare_parameter('mow_cmd_timeout_s', 1.5)
        self.mow_cmd_timeout = self.get_parameter('mow_cmd_timeout_s').value
        self.last_mowEN_cmd_time = None
        self.mow_watchdog_tripped = False

        # --- E-STOP TERÄPORTTI (F30) ---
        # E-stop on pelkkä signaali (Arduinon status-framen parts[0]); mikään
        # Arduino->moottori-polulla ei pakota sitä. Portataan terä pois täällä
        # niin, että e-stop pysäyttää KAIKKI komentajat (myös gamepadin, jolla
        # ei ole omaa ohjelmistoporttia). Salpa pitää terän pois päältä myös
        # e-stopin vapautuksen jälkeen, kunnes komentaja on nähty laskevan
        # enablen alas — pohjassa pidetty gamepad-nappi ei siis käynnistä terää
        # itsestään vapautusreunalla.
        self.estop_blade_latched = False

        # Kesto (s) jonka hoverBtnR1 pidetään päällä yhdellä "pulssilla"
        self.hoverBtnR1_pulse_duration = 0.2
        # Yhden laukauksen ajastin, joka palauttaa napin alas pulssin jälkeen
        self.hoverBtnR1_pulse_timer = None

        # --- TILAUKSET (Ohjaus käskyjä varten) ---
        self.create_subscription(Bool, 'mowMotorEN_cmd', self.mowMotorEN_callback, 10)
        self.create_subscription(Int32, 'mowMotorRPM_set_cmd', self.mowMotorRPM_set_callback, 10)
        self.create_subscription(Bool, 'hoverBtnR1_cmd', self.hoverBtnR1_callback, 10)
        # Pulssi-tilaus: yksi True painallus -> hoverBtnR1 päälle 0.2 s ajaksi
        self.create_subscription(Bool, 'hoverBtnR1_pulse', self.hoverBtnR1_pulse_callback, 10)
        self.create_subscription(Bool, 'varaReleR2_cmd', self.varaReleR2_callback, 10)
        self.create_subscription(Bool, 'lidarPWR_cmd', self.lidarPWR_callback, 10)
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
        self.lidarPWR_pub = self.create_publisher(Bool, 'lidarPWR_status', 10)

        # --- AJASTIN JA LUKUSÄIE ---
        self.timer = self.create_timer(0.1, self.send_to_arduino)
        self.read_thread = threading.Thread(target=self.read_from_arduino, daemon=True)
        self.read_thread.start()

    # Callbackit päivittävät lähetettävän tiedon
    #def hoverBtnR1_callback(self, msg): self.hoverBtnR1_state = 1 if msg.data else 0
    #def varaReleR2_callback(self, msg): self.varaReleR2_state = 1 if msg.data else 0
    #def pwm_callback(self, msg): self.pwm_state = msg.data
    def mowMotorEN_callback(self, msg):
        self.mowMotorEN_State = 1 if msg.data else 0
        self.last_mowEN_cmd_time = time.monotonic()
        if self.mow_watchdog_tripped and msg.data:
            self.get_logger().info(
                "mowMotorEN watchdog: commands resumed — blade allowed again")
        self.mow_watchdog_tripped = False

    def mowMotorRPM_set_callback(self, msg): self.mowMotorRpmSet_state = msg.data
    def hoverBtnR1_callback(self, msg): self.hoverBtnR1_state = 1 if msg.data else 0

    def hoverBtnR1_pulse_callback(self, msg):
        """Painikkeen pulssi: nostaa hoverBtnR1:n ylös ja laskee sen alas
        0.2 s kuluttua. Uusi painallus aloittaa pulssin ajan alusta.
        Ajetaan rclpy:n yksisäikeisellä executorilla, joten ei kilpaile
        send_to_arduino()-ajastimen kanssa."""
        if not msg.data:
            return
        self.hoverBtnR1_state = 1
        # Nollaa mahdollinen aiempi pulssi-ajastin ennen uuden käynnistämistä
        if self.hoverBtnR1_pulse_timer is not None:
            self.hoverBtnR1_pulse_timer.cancel()
            self.destroy_timer(self.hoverBtnR1_pulse_timer)
        self.hoverBtnR1_pulse_timer = self.create_timer(
            self.hoverBtnR1_pulse_duration, self._end_hoverBtnR1_pulse)

    def _end_hoverBtnR1_pulse(self):
        """Yhden laukauksen ajastimen callback: laskee napin alas ja
        tuhoaa ajastimen (rclpy:n ajastimet ovat muuten jaksollisia)."""
        self.hoverBtnR1_state = 0
        if self.hoverBtnR1_pulse_timer is not None:
            self.hoverBtnR1_pulse_timer.cancel()
            self.destroy_timer(self.hoverBtnR1_pulse_timer)
            self.hoverBtnR1_pulse_timer = None

    def varaReleR2_callback(self, msg): self.varaReleR2_state = 1 if msg.data else 0
    def lidarPWR_callback(self, msg): self.lidarPWR_state = 1 if msg.data else 0

    def send_to_arduino(self):
        """Lähettää ohjauskomennot Arduinolle muodossa r1,r2,pwm\n"""
        # Watchdog: sammuta terä jos enable-komentoja ei ole kuulunut
        # mow_cmd_timeout_s sekuntiin (komentaja kuollut tai yhteys poikki).
        if self.mowMotorEN_State == 1:
            if (self.last_mowEN_cmd_time is None or
                    time.monotonic() - self.last_mowEN_cmd_time > self.mow_cmd_timeout):
                if not self.mow_watchdog_tripped:
                    self.mow_watchdog_tripped = True
                    self.get_logger().error(
                        f"mowMotorEN watchdog: no commands for {self.mow_cmd_timeout} s "
                        "— blade shut off")
                self.mowMotorEN_State = 0

        # Guard: don't enable the motor if the RPM setpoint hasn't arrived yet.
        # The enable and RPM commands come from two separate ROS2 topics so the
        # enable callback can fire before the RPM callback, which would start the
        # motor at 0 RPM (minimum speed ~200 RPM) instead of the requested value.
        effective_en = self.mowMotorEN_State
        if effective_en == 1 and self.mowMotorRpmSet_state == 0:
            effective_en = 0

        # F30: e-stop blade gate — backstop that forces the blade off for
        # EVERY commander while the e-stop is active, and LATCHES it off after
        # release until the commander drops enable, so a gamepad button held
        # through an e-stop cycle can't restart the blade on release. The
        # mission and web-UI manager publish mowMotorEN_cmd:false on e-stop
        # themselves, so their latch clears instantly and their resume flows
        # are unchanged.
        if self.eStop_state == 1:
            if not self.estop_blade_latched:
                self.estop_blade_latched = True
                self.get_logger().warn(
                    "mowMotorEN e-stop gate: e-stop active — blade blocked")
            effective_en = 0
        elif self.estop_blade_latched:
            # E-stop released but still latched: keep the blade off until the
            # commander has been seen to drop enable (explicit false, or the
            # keep-alive watchdog above setting mowMotorEN_State = 0).
            if self.mowMotorEN_State == 0:
                self.estop_blade_latched = False
                self.get_logger().info(
                    "mowMotorEN e-stop gate: cleared — enable released")
            else:
                effective_en = 0

        cmd = f"{effective_en},{self.mowMotorRpmSet_state},{self.hoverBtnR1_state},{self.varaReleR2_state},{self.lidarPWR_state}\n"
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
                        
                        if len(parts) == 10:
                            # 0. eStop status
                            # F30: store the state so send_to_arduino's e-stop
                            # blade gate can act on it, not just republish it.
                            self.eStop_state = int(parts[0])
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

                            # 9. Lidar virransyöttö status
                            lidarPWR_msg = Bool()
                            lidarPWR_msg.data = bool(int(parts[9]))
                            self.lidarPWR_pub.publish(lidarPWR_msg)

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