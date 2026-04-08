#!/usr/bin/env python3

import os
import sys
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def yaw_to_quaternion(yaw):
    """Apufunktio yaw-kulman muuntamiseksi quaternioniksi."""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

class MowingManager(Node):
    def __init__(self):
        super().__init__('mowing_manager')
        self.navigator = BasicNavigator()

        # Julkaisija nykyisen ajettavan reitin näyttämiseksi RVizissä
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_publisher = self.create_publisher(Path, 'current_mowing_path', qos_profile)

        self.coverage_data = {}
        # Polku kustomoituun Behavior Treehen, joka sisältää esteistä palautumisen
        self.custom_bt_path = '/home/ros-pi/pi_ws/src/ros2-driver-converted/custom_bt/custom_follow_path.xml'
        self.load_data()

    def load_data(self):
        base_dir = "/home/ros-pi/pi_ws/src/ros2-driver-converted/maps"
        self.paths_file = os.path.join(base_dir, "mow_coverage", "mow_coverage_paths.json")

        if not os.path.exists(self.paths_file):
            self.get_logger().error(f"Reittitiedostoa ei löydy: {self.paths_file}")
            sys.exit(1)

        with open(self.paths_file, 'r') as f:
            self.coverage_data = json.load(f)

    def create_path_msg(self, segment_data, frame_id="map"):
        """Muuttaa JSON-datan standardiksi ROS 2 nav_msgs/Path -viestiksi"""
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for pt in segment_data:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = float(pt["x"])
            pose.pose.position.y = float(pt["y"])
            pose.pose.position.z = 0.0

            qx, qy, qz, qw = yaw_to_quaternion(float(pt["yaw"]))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path.poses.append(pose)

        return path

    def execute_mowing_plan(self, area_names):
        """
        Tämä funktio tekee varsinaisen työn! 
        Tulevaisuudessa Web-UI tai MQTT-kuuntelija voi kutsua suoraan tätä funktiota.
        """
        self.get_logger().info("Odotetaan Nav2-järjestelmän käynnistymistä (ilman AMCL-paikannusta)...")
        # Ohitetaan amcl-paikantimen odotus antamalla localizeriksi bt_navigator
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')

        for alue in area_names:
            area_key = f"{alue}_coverage"
            if area_key not in self.coverage_data:
                self.get_logger().error(f"Aluetta '{alue}' ei löydy tiedostosta. Ohitetaan.")
                continue

            self.get_logger().info(f"\n>>> ALOITETAAN ALUEEN LEIKKUU: {alue.upper()} <<<")
            segments = self.coverage_data[area_key]

            # Varmistetaan segmenttien oikea numerojärjestys (segment_1, segment_2...)
            try:
                segment_keys = sorted(segments.keys(), key=lambda x: int(x.split('_')[1]))
            except Exception:
                segment_keys = list(segments.keys())

            for i, seg_key in enumerate(segment_keys):
                self.get_logger().info(f"\n  -> Suoritetaan: {alue} / {seg_key} ({i+1}/{len(segment_keys)})")

                segment_data = segments[seg_key]
                if not segment_data or len(segment_data) == 0:
                    continue

                path_msg = self.create_path_msg(segment_data)
                start_pose = path_msg.poses[0]

                # --- VAIHE 1: Ajo segmentin alkuun (GoToPose / vapaa suunnittelu) ---
                self.get_logger().info("     1. Siirrytään segmentin aloituspisteeseen (GoToPose)...")
                self.navigator.goToPose(start_pose)

                while not self.navigator.isTaskComplete():
                    # Tässä voisi lähettää esim. edistymispalkin dataa Web-UI/MQTT:lle
                    pass

                result = self.navigator.getResult()
                if result != TaskResult.SUCCEEDED:
                    self.get_logger().error(f"Aloituspisteeseen ajo epäonnistui (Tulos: {result}). Keskeytetään alue.")
                    break  # Siirrytään seuraavaan alueeseen

                # --- VAIHE 2: Reitin seuraaminen (FollowPath) ---
                self.get_logger().info("     2. Robotti aloituspisteessä. Seurataan reittiä (FollowPath)...")
                
                current_path = path_msg
                
                # Pyöritetään silmukkaa niin kauan, kunnes koko segmentin reitti on ajettu
                while len(current_path.poses) > 0:
                    # Julkaistaan nykyinen reitti visualisointia varten
                    self.get_logger().info(f"     3. Julkaistaan reittisegmentti ({len(current_path.poses)} pistettä) visualisointia varten...")
                    self.path_publisher.publish(current_path)

                    self.navigator.followPath(current_path)
                    
                    last_dist_to_goal = None
                    
                    while not self.navigator.isTaskComplete():
                        # Tallennetaan talteen etäisyys maaliin ohituksen laskemista varten
                        feedback = self.navigator.getFeedback()
                        if feedback and hasattr(feedback, 'distance_to_goal'):
                            last_dist_to_goal = feedback.distance_to_goal
                            
                    result = self.navigator.getResult()
                    
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info("     Segmentti valmis!")
                        break  # Siirrytään seuraavaan segmenttiin
                        
                    elif result == TaskResult.FAILED:
                        if last_dist_to_goal is None:
                            self.get_logger().error("     Ei etäisyystietoa. Ohitusta ei voida laskea. Keskeytetään.")
                            break
                            
                        max_jump_attempts = 3
                        jump_increment = 1.0
                        jump_success = False

                        for attempt in range(1, max_jump_attempts + 1):
                            jump_distance = attempt * jump_increment
                            self.get_logger().warn(f"     Este reitillä! Yritetään {jump_distance} metrin ohitusta (yritys {attempt}/{max_jump_attempts})...")
                            
                            target_dist_to_goal = last_dist_to_goal - jump_distance
                            
                            if target_dist_to_goal <= 0:
                                self.get_logger().info("     Este on aivan segmentin lopussa. Lasketaan segmentti valmiiksi.")
                                current_path.poses = []  # Tyhjennetään reitti, jotta pää-silmukka loppuu siististi
                                jump_success = True
                                break
                                
                            # Etsitään reitistä piste, joka on target_dist_to_goal etäisyydellä maalista
                            new_start_idx = -1
                            accumulated_dist = 0.0
                            
                            for idx in range(len(current_path.poses) - 1, 0, -1):
                                p1 = current_path.poses[idx].pose.position
                                p2 = current_path.poses[idx-1].pose.position
                                dist = math.hypot(p1.x - p2.x, p1.y - p2.y)
                                accumulated_dist += dist
                                
                                if accumulated_dist >= target_dist_to_goal:
                                    new_start_idx = idx - 1
                                    break
                                    
                            if new_start_idx == -1:
                                self.get_logger().info("     Ei tarpeeksi reittiä jäljellä ohitukseen. Keskeytetään segmentti.")
                                break
                                
                            # Otetaan testiin pätkä reitistä
                            test_path_poses = current_path.poses[new_start_idx:]
                            new_start_pose = test_path_poses[0]
                            
                            self.get_logger().info("     Suunnitellaan väistöreitti uuteen ohituspisteeseen (GoToPose)...")
                            self.navigator.goToPose(new_start_pose)
                            
                            while not self.navigator.isTaskComplete():
                                pass
                                
                            if self.navigator.getResult() == TaskResult.SUCCEEDED:
                                self.get_logger().info(f"     Ohituspiste ({jump_distance}m) saavutettu! Jatketaan segmentin leikkuuta.")
                                # Tallennetaan onnistunut lyhennetty reitti ja päätetään yrityssilmukka
                                current_path.poses = test_path_poses
                                jump_success = True
                                break
                            else:
                                self.get_logger().warn(f"     Väistö {jump_distance}m pisteeseen epäonnistui (kohde ehkä esteessä).")
                        
                        if not jump_success:
                            self.get_logger().error("     Kaikki ohitusyritykset epäonnistuivat! Keskeytetään tämä segmentti.")
                            break
                        
                    else:
                        self.get_logger().warn(f"     Reitin ajo peruutettiin tai tuntematon virhe ({result}). Keskeytetään.")
                        break

            self.get_logger().info(f"<<< ALUE {alue.upper()} ON VALMIS! >>>")

        self.get_logger().info("\nKaikki pyydetyt alueet on käsitelty. Urakka ohi!")
        # Tyhjennetään reitti RVizistä julkaisemalla tyhjä Path-viesti
        self.get_logger().info("Tyhjennetään reitti visualisoinnista.")
        self.path_publisher.publish(Path())


def main(args=None):
    rclpy.init(args=args)
    manager = MowingManager()
    
    saatavilla = [k.replace("_coverage", "") for k in manager.coverage_data.keys()]
    print("\n=========================================")
    print(" RUOHONLEIKKUUN HALLINTA (MOWING MANAGER)")
    print("=========================================")
    print(f"Tallennetut alueet: {', '.join(saatavilla)}")
    
    # Tämä terminaali-input korvataan tulevaisuudessa esim. MQTT-tilauksella
    valinta = input("\nSyötä leikattavat alueet pilkulla erotettuna (esim. alue1, alue2): ").strip()
    if valinta:
        valitut_alueet = [nimi.strip() for nimi in valinta.split(",") if nimi.strip()]
        manager.execute_mowing_plan(valitut_alueet)
    else:
        print("Ei alueita valittu. Lopetetaan.")
    
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()