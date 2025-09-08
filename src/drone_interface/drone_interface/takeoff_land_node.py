#!/usr/bin/env python3

"""
Nœud ROS2 minimal pour décollage et atterrissage via MAVROS
Gère les erreurs et timeouts selon les recommandations officielles
"""


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from std_msgs.msg import Float64

from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State

from geometry_msgs.msg import PoseStamped


import time


class TakeoffLandNode(Node):
    def __init__(self):
        super().__init__('takeoff_land_node')

        # Etat du noeud
        self.mavros_state = None
        self.local_pose = None
        self.rel_alt = None

        # Client pour le décollage
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        # Client pour l'atterrissage
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        
        # Souscription aux commandes
        self.create_subscribers()
        # Publication de topic
        self.create_publishers()

        


        

    
    def create_subscribers(self):
        
        self.takeoff_sub = self.create_subscription(
            Bool,
            '/takeoff_control/takeoff_cmd',
            self.takeoff_callback,
            10
        )
        self.land_sub = self.create_subscription(
            Bool,
            '/takeoff_control/land_cmd',
            self.land_callback,
            10
        )

        self.mavros_state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.mavros_state_callback,
            10
        )

        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.local_pose_callback,
            self.sensor_qos
        )
        self.rel_alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.rel_alt_callback,
            self.sensor_qos
        )

    def create_publishers(self):

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10
        )

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def rel_alt_callback(self, msg):
        self.rel_alt = msg.data


    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def create_setpoints(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        # Publier plusieurs setpoints sur 1 seconde
        for _ in range(20):
            pose.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(pose)
            time.sleep(0.05)



    def takeoff_callback(self, msg):
        try:
            if not msg.data:
                return
            self.get_logger().info('🛫 Décollage demandé')

            # S'assurer que le drone est connecté, armé et en mode GUIDED
            if self.mavros_state is None:
                self.get_logger().error('⏳ Attente du message /mavros/state')
                return
            if not self.mavros_state.connected:
                self.get_logger().error('❌ MAVROS non connecté')
                return
            if self.mavros_state.mode != 'GUIDED':
                self.get_logger().error('⚠️ Le mode doit être GUIDED')
                return
            if not self.mavros_state.armed:
                self.get_logger().error('🔒 Le drone n\'est pas armé')
                return

            # Vérification position locale
            if self.local_pose is None:
                self.get_logger().error('⏳ Attente du message /mavros/local_position/pose')
                return
            pos = self.local_pose.pose.position
            if any([str(pos.x) == 'nan', str(pos.y) == 'nan', str(pos.z) == 'nan']):
                self.get_logger().error('❌ Position locale non valide (NaN)')
                return

            # Vérification altitude relative
            if self.rel_alt is None:
                self.get_logger().error('⏳ Attente du message /mavros/global_position/rel_alt')
                return
            if str(self.rel_alt) == 'nan' or self.rel_alt == 0.0:
                self.get_logger().error('❌ Altitude relative non valide (NaN ou 0.0)')
                return

            # Publier des setpoints avant le décollage (protection MAVROS/ArduPilot)
            self.create_setpoints()

            if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('⏳ Service takeoff non disponible')
                return
            req = CommandTOL.Request()
            req.min_pitch = 0.0
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            req.altitude = 8.0  # Altitude simple
            future = self.takeoff_client.call_async(req)
            future.add_done_callback(self.takeoff_response)
        except Exception as e:
            self.get_logger().error(f'⚠️ Exception dans takeoff_callback: {e}')

    def takeoff_response(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('✅ Décollage réussi')
            else:
                self.get_logger().error('❌ Décollage refusé')
        except Exception as e:
            self.get_logger().error(f'⚠️ Erreur takeoff: {e}')

    def land_callback(self, msg):
        try:
            if not msg.data:
                return
            self.get_logger().info('🛬 Atterrissage demandé')

            # S'assurer que le drone est connecté, armé et en mode GUIDED
            if self.mavros_state is None:
                self.get_logger().error('⏳ Attente du message /mavros/state')
                return
            if not self.mavros_state.connected:
                self.get_logger().error('❌ MAVROS non connecté')
                return
            if self.mavros_state.mode != 'GUIDED':
                self.get_logger().error('⚠️ Le mode doit être GUIDED')
                return
            if not self.mavros_state.armed:
                self.get_logger().error('🔒 Le drone n\'est pas armé')
                return

            # Publier des setpoints avant l'atterrissage (protection MAVROS/ArduPilot)
            self.create_setpoints()

            if not self.land_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('⏳ Service land non disponible')
                return
            req = CommandTOL.Request()
            req.min_pitch = 0.0
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            req.altitude = 0.0
            future = self.land_client.call_async(req)
            future.add_done_callback(self.land_response)
        except Exception as e:
            self.get_logger().error(f'⚠️ Exception dans land_callback: {e}')

    def land_response(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('✅ Atterrissage réussi')
            else:
                self.get_logger().error('❌ Atterrissage refusé')
        except Exception as e:
            self.get_logger().error(f'⚠️ Erreur land: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLandNode()
    node.get_logger().info('Nœud takeoff/land prêt')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()