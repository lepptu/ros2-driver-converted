#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import os
import json
import threading

class BoundaryRecorder(Node):
    def __init__(self):
        super().__init__('boundary_recorder')
        
        # Kuunnellaan globaalia EKF:ää (joka yhdistää GPS:n ja pyörät)
        self.sub = self.create_subscription(Odometry, '/odometry/global', self.odom_callback, 10)
        
        # Tilamuuttujat
        self.state = "IDLE"  # Voi olla: IDLE, RECORDING_OUTLINE, RECORDING_HOLE
        self.current_points = []
        self.min_dist = 0.0
        self.last_x = None
        self.last_y = None
    def start_recording(self, mode):
        self.state = mode
        self.current_points = []
        self.last_x = None
        self.last_y = None
        # Säädetään tarkkuus tilan mukaan
        if mode == "RECORDING_OUTLINE":
            self.min_dist = 0.5  # 50 cm
        elif mode == "RECORDING_HOLE":
            self.min_dist = 0.1  # 10 cm
            
        self.get_logger().info(f"Aloitettiin nauhoitus tilassa: {mode} ({self.min_dist}m välein)")

    def stop_recording(self):
        self.state = "IDLE"
        
        # 1. Aloitus- ja lopetuspisteen automaattinen sulkeminen (Polygon Closure)
        if len(self.current_points) > 0:
            ensimmainen = self.current_points[0].copy()
            self.current_points.append(ensimmainen)
            
        self.get_logger().info(f"Nauhoitus pysäytetty. Pisteitä kertyi: {len(self.current_points)}")
        return self.current_points

    def odom_callback(self, msg):
        if self.state == "IDLE":
            return
            
        # Haetaan robotin nykyinen X ja Y sijainti kartalla
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Jos tämä on ensimmäinen piste, tallennetaan se heti
        if self.last_x is None:
            self.current_points.append({"x": round(x, 4), "y": round(y, 4)})
            self.last_x = x
            self.last_y = y
            self.get_logger().info(f"Aloituspiste tallennettu: X={x:.2f}, Y={y:.2f}")
            return
            
        # Lasketaan etäisyys edelliseen
        dist = math.hypot(x - self.last_x, y - self.last_y)
        
        # Tallennetaan vain, jos ollaan liikuttu tarpeeksi
        if dist >= self.min_dist:
            self.current_points.append({"x": round(x, 4), "y": round(y, 4)})
            self.last_x = x
            self.last_y = y
            self.get_logger().info(f"Piste tallennettu [{len(self.current_points)}]: X={x:.2f}, Y={y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = BoundaryRecorder()
    
    # Käynnistetään ROS-solmu taustasäikeeseen, jotta input() kyselyt toimivat
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        print("\n=======================================================")
        print(" PIHAN RAJOJEN JA ESTEIDEN NAUHOITUS (MOW AREAS)")
        print("=======================================================")
        
        area_name = input("Anna alueen nimi (esim. alue1): ").strip()
        if not area_name:
            print("Virhe: Nimi ei voi olla tyhjä.")
            raise KeyboardInterrupt
            
        # --- 1. ULKORAJAN NAUHOITUS ---
        print(f"\nSiirrä robotti alueen '{area_name}' ulkorajan aloituspisteeseen.")
        input("Paina ENTER kun olet valmis aloittamaan ulkorajan nauhoituksen...")
        
        node.start_recording("RECORDING_OUTLINE")
        input("\nAja nyt ulkorajaa pitkin ympäri. Paina ENTER kun olet takaisin aloituspisteessä...")
        outline_points = node.stop_recording()
        
        # --- 2. KEEPOUT ZONET (REIÄT) ---
        holes = []
        while True:
            ans = input("\nLisätäänkö keepout zoneja (kierrettäviä esteitä)? y/n: ").strip().lower()
            if ans != 'y':
                break
                
            ready = input("Oletko keepout zonen aloituspisteessä? y/n: ").strip().lower()
            if ready == 'y':
                node.start_recording("RECORDING_HOLE")
                action = input("\nAja nyt esteen ympäri. Paina ENTER kun olet takaisin esteen aloituspisteessä, tai 'r' peruttaaksesi: ").strip().lower()
                hole_points = node.stop_recording()
                
                # 2. Peruutustoiminto (Undo) keepout zonelle
                if action == 'r':
                    print("--> Keepout-alueen nauhoitus hylättiin! Voit yrittää kyseistä aluetta uudelleen.")
                else:
                    holes.append(hole_points)
                    print(f"Keepout-alue tallennettu ({len(hole_points)} pistettä).")
            else:
                print("Aja ensin esteen viereen ja yritä sitten uudelleen.")
                
        # --- 3. TALLENNUS JSON-TIEDOSTOON ---
        base_dir = "/home/ros-pi/pi_ws/src/ros2-driver-converted/maps"
        out_dir = os.path.join(base_dir, "mow_area")
        os.makedirs(out_dir, exist_ok=True)
        json_path = os.path.join(out_dir, "mow_areas.json")
        
        # Ladataan vanhat alueet jos tiedosto on jo olemassa
        if os.path.exists(json_path):
            with open(json_path, 'r') as f:
                try:
                    mow_data = json.load(f)
                except json.JSONDecodeError:
                    mow_data = {"mow_areas": {}}
        else:
            mow_data = {"mow_areas": {}}
            
        if "mow_areas" not in mow_data:
            mow_data["mow_areas"] = {}
            
        # Lisätään uusi alue
        mow_data["mow_areas"][area_name] = {
            "outline": outline_points,
            "holes": holes
        }
        
        # Tallennetaan tiedosto
        with open(json_path, 'w') as f:
            json.dump(mow_data, f, indent=2)
            
        print(f"\nVALMIS! Alue '{area_name}' tallennettu onnistuneesti tiedostoon:")
        print(f"{json_path}")
        print(f" - Ulkorajan pisteitä: {len(outline_points)}")
        print(f" - Keepout-alueita: {len(holes)}")
        
    except KeyboardInterrupt:
        print("\nNauhoitus peruutettu.")
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()