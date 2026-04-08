#!/usr/bin/env python3

import os
import sys
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon as RosPolygon, Point32
from shapely.geometry import Polygon as ShapelyPolygon
from slic3r_coverage_planner.srv import PlanPath

class Slic3rCoverageGenerator(Node):
    def __init__(self):
        super().__init__('slic3r_coverage_generator')
        self.cli = self.create_client(PlanPath, '/slic3r_coverage_planner/plan_path')
        
    def wait_for_service(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Odotetaan slic3r_coverage_planner -palvelua...')
            
    def plan_path(self, outline_points, hole_polygons, distance=0.35, angle=0.0):
        req = PlanPath.Request()
        req.fill_type = PlanPath.Request.FILL_LINEAR
        req.angle = float(angle)
        req.distance = float(distance)
        req.outer_offset = 0.0  # Hoidetaan Shapelylla
        req.outline_count = 1
        req.outline_overlap_count = 0
        
        req.outline = RosPolygon()
        for pt in outline_points:
            p = Point32()
            p.x = float(pt['x'])
            p.y = float(pt['y'])
            p.z = 0.0
            req.outline.points.append(p)
            
        req.holes = []
        for hole_points in hole_polygons:
            hole_poly_msg = RosPolygon()
            for pt in hole_points:
                p = Point32()
                p.x = float(pt['x'])
                p.y = float(pt['y'])
                p.z = 0.0
                hole_poly_msg.points.append(p)
            req.holes.append(hole_poly_msg)

        self.get_logger().info('Pyydetään Slic3r:lta reittiä...')
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            return response.paths
        except Exception as e:
            self.get_logger().error(f'Palvelukutsu epäonnistui: {e}')
            return None

def main(args=None):
    # Määritetään tiedostopolut (oletetaan, että ajetaan pi_ws/ -kansiosta tai vastaavasta)
    base_dir = "/home/ros-pi/pi_ws/src/ros2-driver-converted/maps"
    in_file = os.path.join(base_dir, "mow_area", "mow_areas.json")
    out_dir = os.path.join(base_dir, "mow_coverage")
    
    # Varmistetaan, että ulostulokansio on olemassa
    os.makedirs(out_dir, exist_ok=True)
    
    # 1. Pyydetään alueen nimi
    print("=========================================")
    print(" SLIC3R COVERAGE PATH GENERATOR")
    print("=========================================")
    area_name = input("Anna alueen nimi (esim. alue1): ").strip()
    
    if not area_name:
        print("Virhe: Nimi ei voi olla tyhjä.")
        sys.exit(1)
        
    # 2. Luetaan JSON-tiedosto
    if not os.path.exists(in_file):
        print(f"Virhe: Lähdetiedostoa ei löytynyt polusta: {in_file}")
        sys.exit(1)
        
    with open(in_file, 'r') as f:
        try:
            mow_data = json.load(f)
        except json.JSONDecodeError as e:
            print(f"Virhe JSON-tiedoston lukemisessa: {e}")
            sys.exit(1)
            
    # 3. Tarkistetaan löytyykö alue
    if "mow_areas" not in mow_data or area_name not in mow_data["mow_areas"]:
        print(f"Virhe: Aluetta '{area_name}' ei löytynyt tiedostosta {in_file}.")
        print(f"Löytyneet alueet: {list(mow_data.get('mow_areas', {}).keys())}")
        sys.exit(1)
        
    area_data = mow_data["mow_areas"][area_name]
    
    # Tarkistetaan, onko data vanhassa (lista) vai uudessa (objekti) muodossa
    if isinstance(area_data, list):
        area_points = area_data
        hole_definitions = []
    elif isinstance(area_data, dict) and 'outline' in area_data:
        area_points = area_data['outline']
        hole_definitions = area_data.get('holes', [])
    else:
        print(f"Virhe: Alueen '{area_name}' data on tuntemattomassa muodossa.")
        sys.exit(1)

    print(f"Alue '{area_name}' ladattu. Ulkorajan pisteitä: {len(area_points)}, reikiä: {len(hole_definitions)}")

    # Kysytään asetukset käyttäjältä
    angle_str = input("Anna leikkuukulma asteina (oletus 0.0): ").strip()
    angle = float(angle_str) if angle_str else 0.0
    
    distance_str = input("Anna leikkuuleveys metreinä (oletus 0.35): ").strip()
    distance = float(distance_str) if distance_str else 0.35
    
    buffer_str = input("Anna turvaetäisyys reunoista metreinä (oletus 0.2): ").strip()
    buffer_dist = float(buffer_str) if buffer_str else 0.2

    # TAIKATEMPPU 1: Kutistetaan ulkorajaa Shapelylla (buffer)
    coords = [(pt['x'], pt['y']) for pt in area_points]
    poly = ShapelyPolygon(coords)
    safe_poly = poly.buffer(0).buffer(-buffer_dist)
    
    if safe_poly.is_empty:
        print("Virhe: Turvaetäisyys on liian suuri, alue kutistui olemattomiin!")
        sys.exit(1)
        
    if safe_poly.geom_type == 'MultiPolygon':
        # Jos alue jakautui useaan osaan, otetaan niistä suurin
        safe_poly = max(safe_poly.geoms, key=lambda a: a.area)
        
    buffered_points = [{'x': c[0], 'y': c[1]} for c in safe_poly.exterior.coords]
    print(f"Ulkorajaa pienennetty {buffer_dist}m. Uusia koordinaatteja: {len(buffered_points)}")

    # TAIKATEMPPU 2: Laajennetaan reikiä turvaetäisyydellä
    buffered_hole_polygons = []
    for hole_points in hole_definitions:
        hole_coords = [(pt['x'], pt['y']) for pt in hole_points]
        hole_poly = ShapelyPolygon(hole_coords)
        # Laajennetaan reikää, jotta robotti kiertää sen kauempaa
        safe_hole_poly = hole_poly.buffer(buffer_dist) 
        buffered_hole_points = [{'x': c[0], 'y': c[1]} for c in safe_hole_poly.exterior.coords]
        buffered_hole_polygons.append(buffered_hole_points)

    # 4. Käynnistetään ROS 2 ja haetaan reitti
    rclpy.init(args=args)
    generator = Slic3rCoverageGenerator()
    generator.wait_for_service()
    
    # Pyydetään reitti pienennettyllä ulkorajalla ja laajennetuilla rei'illä
    paths = generator.plan_path(buffered_points, buffered_hole_polygons, distance=distance, angle=angle)
    
    if not paths:
        print("Reitin laskenta epäonnistui.")
        generator.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
        
    print(f"Slic3r palautti {len(paths)} segmenttiä.")
    
    # 5. Muotoillaan tulos ja tallennetaan
    area_coverage_data = {}
    
    for idx, path_segment in enumerate(paths):
        segment_points = []
        poses = path_segment.path.poses
        for i in range(len(poses)):
            x = poses[i].pose.position.x
            y = poses[i].pose.position.y
            
            # Suunnan (yaw) laskeminen seuraavaan pisteeseen
            if i < len(poses) - 1:
                nx = poses[i+1].pose.position.x
                ny = poses[i+1].pose.position.y
                yaw = math.atan2(ny - y, nx - x)
            else:
                # Viimeinen piste: pidetään sama suunta kuin edellisessä
                if i > 0:
                    px = poses[i-1].pose.position.x
                    py = poses[i-1].pose.position.y
                    yaw = math.atan2(y - py, x - px)
                else:
                    yaw = 0.0

            segment_points.append({
                "x": round(x, 4),
                "y": round(y, 4),
                "yaw": round(yaw, 4)
            })
        area_coverage_data[f"segment_{idx+1}"] = segment_points
        
    out_file_path = os.path.join(out_dir, "mow_coverage_paths.json")
    
    # Ladataan olemassa oleva tiedosto, jotta ei ylikirjoiteta vanhoja alueita
    if os.path.exists(out_file_path):
        with open(out_file_path, 'r') as f:
            try:
                all_coverage_data = json.load(f)
            except json.JSONDecodeError:
                all_coverage_data = {}
    else:
        all_coverage_data = {}
        
    all_coverage_data[f"{area_name}_coverage"] = area_coverage_data
    
    with open(out_file_path, 'w') as f:
        json.dump(all_coverage_data, f, indent=2)
        
    print(f"VALMIS! Reitti tallennettu onnistuneesti tiedostoon:")
    print(out_file_path)
    
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
