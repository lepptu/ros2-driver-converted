#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import os

class BoundaryRecorder(Node):
    def __init__(self):
        super().__init__('boundary_recorder')
        
        # Kuunnellaan globaalia EKF:ää (joka yhdistää GPS:n ja pyörät)
        self.sub = self.create_subscription(Odometry, '/odometry/global', self.odom_callback, 10)
        
        # Tallennetaan tiedosto suoraan kotikansioon, jotta se on helppo löytää
        self.filename = os.path.expanduser('~/pi_ws/pihan_rajat.csv')
        self.min_dist = 0.5  # Tallenna uusi piste 0.5 metrin välein
        self.last_x = None
        self.last_y = None
        
        # Tyhjennetään/luodaan uusi tiedosto ja kirjoitetaan otsikot
        with open(self.filename, 'w') as f:
            f.write('x,y\n')
        
        self.get_logger().info("=== RAJOJEN TALLENNUS KÄYNNISTETTY ===")
        self.get_logger().info(f"Tiedosto: {self.filename}")
        self.get_logger().info(f"Tallentaa uuden pisteen {self.min_dist} metrin välein. Aja robottia nyt!")

    def odom_callback(self, msg):
        # Haetaan robotin nykyinen X ja Y sijainti kartalla
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Jos tämä on ensimmäinen piste, tallennetaan se heti
        if self.last_x is None:
            self.save_point(x, y)
            return
            
        # Lasketaan etäisyys edelliseen tallennettuun pisteeseen (Pythagoraan lause)
        dist = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
        
        # Tallennetaan vain, jos ollaan liikuttu tarpeeksi
        if dist >= self.min_dist:
            self.save_point(x, y)

    def save_point(self, x, y):
        with open(self.filename, 'a') as f:
            f.write(f"{x:.3f},{y:.3f}\n")
        self.last_x = x
        self.last_y = y
        self.get_logger().info(f"Piste tallennettu -> X: {x:.3f}, Y: {y:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = BoundaryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Tallennus lopetettu. Tiedosto on valmis!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()