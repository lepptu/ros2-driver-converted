#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int32
from rcl_interfaces.msg import SetParametersResult # Tarvitaan callbackia varten

class JoyToArduino(Node):
    def __init__(self):
        super().__init__('joy_to_arduino')
        
        # Ladataan parametrit yaml-tiedostosta
        #self.declare_parameter('action_button', 13)
        #self.declare_parameter('motor_speed_value', 100)
        self.declare_parameter('mowEnable_button', 13)
        self.declare_parameter('mowRpmSET_value', 100)
        self.declare_parameter('hoverBtnR1_button', 7)
        self.declare_parameter('varaReleR2_button', 6)
        self.declare_parameter('lidarPWR_button', 3)

        # Haetaan arvot (esim. action_button = 3)
        #self.btn_idx = self.get_parameter('action_button').value
        #self.speed_val = self.get_parameter('motor_speed_value').value
        self.mowEnable_idx = self.get_parameter('mowEnable_button').value
        self.mowRpmSET_val = self.get_parameter('mowRpmSET_value').value
        self.hoverBtnR1_idx = self.get_parameter('hoverBtnR1_button').value
        self.varaReleR2_idx = self.get_parameter('varaReleR2_button').value
        self.lidarPWR_idx = self.get_parameter('lidarPWR_button').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        # Track previous button states so we only publish mow motor commands
        # on state CHANGES (edge-triggered).  Publishing false on every joystick
        # tick at joystick rate drowns out the autonomous keep-alive signal.
        self._prev_mowEn = False
        self._prev_lidarPWR = False
        self._lidarPWR_state = False

        # Julkaisijat
        #self.rele1_pub = self.create_publisher(Bool, 'rele1_cmd', 10)
        #self.rele2_pub = self.create_publisher(Bool, 'rele2_cmd', 10)
        #self.pwm_pub = self.create_publisher(Int32, 'motor_pwm_cmd', 10)
        self.mowMotorEN_pub = self.create_publisher(Bool, 'mowMotorEN_cmd', 10)
        self.mowMotorRpmSET_pub = self.create_publisher(Int32, 'mowMotorRPM_set_cmd', 10)
        self.hoverBtnR1_pub = self.create_publisher(Bool, 'hoverBtnR1_cmd', 10)
        self.varaReleR2_pub = self.create_publisher(Bool, 'varaReleR2_cmd', 10)
        self.lidarPWR_pub = self.create_publisher(Bool, 'lidarPWR_cmd', 10)

        # Tilaaja ohjaimen raakadatalle (/joy)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.get_logger().info(f"Kuunnellaan /joy topicia. Nappi indeksissä {self.mowEnable_idx} aktivoi leikkausmoottorin.")

    def parameter_callback(self, params):
        """Tätä kutsutaan automaattisesti, kun parametreja muutetaan."""
        for param in params:
            if param.name == 'mowEnable_button':
                self.mowEnable_idx = param.value
                self.get_logger().info(f"Päivitetty mowEnable_button: {param.value}")
            
            elif param.name == 'mowRpmSET_value':
                self.mowRpmSET_val = param.value
                self.get_logger().info(f"Päivitetty mowRpmSET_value: {param.value}")
            
            elif param.name == 'hoverBtnR1_button':
                self.hoverBtnR1_idx = param.value
                self.get_logger().info(f"Päivitetty hoverBtnR1_button: {param.value}")
            
            elif param.name == 'varaReleR2_button':
                self.varaReleR2_idx = param.value
                self.get_logger().info(f"Päivitetty varaReleR2_button: {param.value}")

            elif param.name == 'lidarPWR_button':
                self.lidarPWR_idx = param.value
                self.get_logger().info(f"Päivitetty lidarPWR_button: {param.value}")
        
        return SetParametersResult(successful=True)


    def joy_callback(self, msg):
        # Varmistetaan ensin, että taulukossa on tarpeeksi alkioita, jottei ohjelma kaadu
        #if len(msg.buttons) > self.btn_idx:
        if len(msg.buttons) > self.mowEnable_idx:

            # msg.buttons[3] lukee taulukon 4. arvon. Jos se on 1, is_pressed = True
            mowEn_is_pressed = (msg.buttons[self.mowEnable_idx] == 1)
            hoverBtnR1_is_pressed = (msg.buttons[self.hoverBtnR1_idx] == 1)
            varaReleR2_is_pressed = (msg.buttons[self.varaReleR2_idx] == 1)
            lidarPWR_is_pressed = (msg.buttons[self.lidarPWR_idx] == 1)

            # Mow motor: only publish when button state changes so we don't
            # flood the topic with false and override the autonomous keep-alive.
            if mowEn_is_pressed != self._prev_mowEn:
                self._prev_mowEn = mowEn_is_pressed
                mowEn_msg = Bool()
                mowEn_msg.data = mowEn_is_pressed
                self.mowMotorEN_pub.publish(mowEn_msg)
                mowRpm_msg = Int32()
                mowRpm_msg.data = self.mowRpmSET_val if mowEn_is_pressed else 0
                self.mowMotorRpmSET_pub.publish(mowRpm_msg)

            hoverBtnR1_msg = Bool()
            hoverBtnR1_msg.data = hoverBtnR1_is_pressed

            varaReleR2_msg = Bool()
            varaReleR2_msg.data = varaReleR2_is_pressed

            # lidarPWR: toggle on rising edge (press, not hold)
            if lidarPWR_is_pressed and not self._prev_lidarPWR:
                self._lidarPWR_state = not self._lidarPWR_state
                lidarPWR_msg = Bool()
                lidarPWR_msg.data = self._lidarPWR_state
                self.lidarPWR_pub.publish(lidarPWR_msg)
                self.get_logger().info(f"lidarPWR toggled: {self._lidarPWR_state}")
            self._prev_lidarPWR = lidarPWR_is_pressed

            # Julkaistaan viestit ROS2-verkkoon
            self.hoverBtnR1_pub.publish(hoverBtnR1_msg)
            self.varaReleR2_pub.publish(varaReleR2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToArduino()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()