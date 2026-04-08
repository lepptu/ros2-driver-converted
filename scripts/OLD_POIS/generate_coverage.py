#!/usr/bin/env python3

import csv
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.affinity import rotate

class SimpleCoveragePlanner(Node):
    def __init__(self):
        super().__init__('simple_coverage_planner')
        
        # Julkaistaan reitti RViziä varten
        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)
        
        # --- ASETUKSET ---
        self.tiedosto_in = 'pihan_rajat.csv'  # Luettava tiedosto
        self.tiedosto_out = 'reitti.csv'      # Mihin reitti tallennetaan
        
        self.leikkuuleveys = 0.35             # Viivojen väli metreinä
        self.turva_etaisyys = 0.25            # Kuinka kauas ulkorajasta jäädään
        self.ajo_kulma = 0.0                  # Siksakin kulma asteina
        # -----------------
        
        self.path_msg = None

        self.suunnittele_reitti()

    def lue_rajat(self):
        pisteet = []
        try:
            with open(self.tiedosto_in, 'r') as f:
                lines = f.readlines()
                for line in lines[1:]: # Ohitetaan otsikko "x,y"
                    x, y = line.strip().split(',')
                    pisteet.append((float(x), float(y)))
            return pisteet
        except Exception as e:
            self.get_logger().error(f'Virhe tiedoston luvussa: {e}')
            return None

    def suunnittele_reitti(self):
        rajapisteet = self.lue_rajat()
        if not rajapisteet or len(rajapisteet) < 3:
            return

        alkuperainen_piha = Polygon(rajapisteet)
        
        # TAIKATEMPPU 1: Korjataan GPS-virheistä johtuvat risteämät
        alkuperainen_piha = alkuperainen_piha.buffer(0)
        
        # Pienennetään turvaetäisyyden verran
        turvallinen_piha = alkuperainen_piha.buffer(-self.turva_etaisyys)
        
        if turvallinen_piha.is_empty:
            self.get_logger().error("Piha on liian pieni tai turvaetäisyys on liian suuri!")
            return

        kaannetty_piha = rotate(turvallinen_piha, -self.ajo_kulma, origin='centroid')
        minx, miny, maxx, maxy = kaannetty_piha.bounds
        
        y = miny
        viivat = []
        while y <= maxy:
            viiva = LineString([(minx - 1, y), (maxx + 1, y)])
            leikattu_viiva = viiva.intersection(kaannetty_piha)
            
            if not leikattu_viiva.is_empty:
                # Otetaan kaikki pätkät talteen
                if isinstance(leikattu_viiva, MultiLineString):
                    for segment in leikattu_viiva.geoms:
                        viivat.append(segment)
                else:
                    viivat.append(leikattu_viiva)
            y += self.leikkuuleveys

        # --- TAIKATEMPPU 2: Älykäs yhdistäminen (lähin piste) ---
        if not viivat:
            return

        reitti_pisteet_kaannettyna = []
        
        nykyinen_viiva = viivat.pop(0)
        coords = list(nykyinen_viiva.coords)
        reitti_pisteet_kaannettyna.extend(coords)
        nykyinen_piste = coords[-1]
        
        while viivat:
            paras_etaisyys = float('inf')
            paras_indeksi = -1
            kaanna_viiva = False
            
            for i, viiva in enumerate(viivat):
                c = list(viiva.coords)
                alku = c[0]
                loppu = c[-1]
                
                dist_alku = math.hypot(alku[0] - nykyinen_piste[0], alku[1] - nykyinen_piste[1])
                dist_loppu = math.hypot(loppu[0] - nykyinen_piste[0], loppu[1] - nykyinen_piste[1])
                
                if dist_alku < paras_etaisyys:
                    paras_etaisyys = dist_alku
                    paras_indeksi = i
                    kaanna_viiva = False
                    
                if dist_loppu < paras_etaisyys:
                    paras_etaisyys = dist_loppu
                    paras_indeksi = i
                    kaanna_viiva = True
                    
            seuraava_viiva = viivat.pop(paras_indeksi)
            c = list(seuraava_viiva.coords)
            if kaanna_viiva:
                c.reverse()
                
            reitti_pisteet_kaannettyna.extend(c)
            nykyinen_piste = c[-1]

        lopullinen_reitti = rotate(LineString(reitti_pisteet_kaannettyna), self.ajo_kulma, origin='centroid')
        
        self.tallenna_ja_julkaise(list(lopullinen_reitti.coords))

    def tallenna_ja_julkaise(self, pisteet):
        with open(self.tiedosto_out, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for p in pisteet:
                writer.writerow([round(p[0], 4), round(p[1], 4)])
                
        self.get_logger().info(f'Reitti laskettu! {len(pisteet)} reittipistettä tallennettu tiedostoon: {self.tiedosto_out}')

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        for x, y in pisteet:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.path_msg.poses.append(pose)

        self.timer = self.create_timer(1.0, self.julkaise_silmukka)
        self.get_logger().info('Reittiä julkaistaan RViziin (topic: /coverage_path).')
        self.get_logger().info('Paina Ctrl+C kun haluat sulkea skriptin.')

    def julkaise_silmukka(self):
        if self.path_msg:
            now = self.get_clock().now().to_msg()
            self.path_msg.header.stamp = now
            for pose in self.path_msg.poses:
                pose.header.stamp = now
            self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCoveragePlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()