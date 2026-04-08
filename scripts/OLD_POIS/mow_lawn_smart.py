#!/usr/bin/env python3

import csv
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus

class SmartMowerController(Node):
    def __init__(self):
        super().__init__('smart_mower_controller')
        self.action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.reittitiedosto = 'reitti.csv'

    def laheta_reitti(self):
        pisteet_xy = []
        
        # 1. Luetaan pelkät X ja Y koordinaatit ensin listaan
        try:
            with open(self.reittitiedosto, 'r') as f:
                lines = f.readlines()
                for line in lines[1:]: # Ohitetaan otsikko
                    x, y = line.strip().split(',')
                    pisteet_xy.append((float(x), float(y)))
        except Exception as e:
            self.get_logger().error(f'Virhe tiedoston luvussa: {e}')
            return

        pisteet = []
        
        # 2. Tehdään koordinaateista PoseStamped-viestejä ja lasketaan niille OIKEEA SUUNTA
        for i in range(len(pisteet_xy)):
            x, y = pisteet_xy[i]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Lasketaan suuntakulma (yaw) seuraavaan pisteeseen
            if i < len(pisteet_xy) - 1:
                seuraava_x, seuraava_y = pisteet_xy[i+1]
                yaw = math.atan2(seuraava_y - y, seuraava_x - x)
            else:
                # Viimeinen piste: pidetään sama suunta kuin edellisessä
                edellinen_x, edellinen_y = pisteet_xy[i-1]
                yaw = math.atan2(y - edellinen_y, x - edellinen_x)
                
            # Muutetaan kulma (yaw) ROS 2:n ymmärtämäksi kvaternioksi
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            pisteet.append(pose)

        self.get_logger().info(f'Luettiin {len(pisteet)} reittipistettä ja laskettiin niille ajosuunnat.')
        self.action_client.wait_for_server()

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = pisteet
        goal_msg.behavior_tree = '' 

        self.get_logger().warn('Lähetetään älykäs reitti. Robotti lähtee liikkeelle!')
        
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 hylkäsi reitin heti alussa!')
            return

        self.get_logger().info('Reitti hyväksytty! Ajo käynnissä...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        
        # Tarkistetaan oikeasti, onnistuiko ajo vai keskeytyikö se!
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Hienoa työtä! Koko alue on leikattu onnistuneesti.')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Ajo keskeytyi (ABORTED)! Robotti ei löytänyt reittiä seuraavaan pisteeseen.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning('Ajo peruutettiin (CANCELED).')
        else:
            self.get_logger().info(f'Ajo päättyi tilaan: {status}')
            
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SmartMowerController()
    node.laheta_reitti()
    rclpy.spin(node)

if __name__ == '__main__':
    main()