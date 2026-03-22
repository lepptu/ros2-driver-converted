#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32
import csv
import os

class BoundaryPublisher(Node):
    def __init__(self):
        super().__init__('boundary_publisher')
        # Julkaistaan yard_boundary -nimiseen topicciin
        self.publisher_ = self.create_publisher(PolygonStamped, 'yard_boundary', 10)
        
        # Julkaistaan sekunnin välein, jotta RViz löytää sen varmasti
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.boundary_msg = PolygonStamped()
        self.boundary_msg.header.frame_id = 'map'  # Varmista että tämä on 'map'
        
        # LAITA TÄHÄN OIKEA POLKU CSV-TIEDOSTOOSI!
        csv_path = '/home/ros-pi/pi_ws/pihan_rajat.csv' 
        self.load_csv(csv_path)

    def load_csv(self, filename):
        if not os.path.exists(filename):
            self.get_logger().error(f"Tiedostoa {filename} ei löydy!")
            return
            
        try:
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                next(reader, None)
                for row in reader:
                    # Oletetaan, että sarake 0 on X ja sarake 1 on Y
                    pt = Point32()
                    pt.x = float(row[0])
                    pt.y = float(row[1])
                    pt.z = 0.0
                    self.boundary_msg.polygon.points.append(pt)
            self.get_logger().info(f"Ladattiin {len(self.boundary_msg.polygon.points)} pistettä pihan rajasta.")
        except Exception as e:
            self.get_logger().error(f"Virhe CSV-luennassa: {e}")

    def timer_callback(self):
        self.boundary_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.boundary_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BoundaryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()