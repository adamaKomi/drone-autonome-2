import math
import time
from typing import Optional

import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header

class DroneInterface(Node):
    def __init__(self):
        super().__init__('drone_interface')

        qos = QoSProfile(depth=10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos)

        # Clients MAVROS
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')

        # Timers
        self.timer = self.create_timer(0.1, self._heartbeat)  # 10 Hz publication si une cible active existe
        self.active_target: Optional[PoseStamped] = None # dernière cible publiée

        self.get_logger().info('drone_interface prêt. Services: arm(), set_mode(), takeoff_alt(), goto_local()')

    # ---------- Helpers ----------
    def _call_arm(self, value: bool) -> bool:
        if not self.arm_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service /mavros/cmd/arming indisponible')
            return False
        req = CommandBool.Request(value=value)
        future = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            res = future.result()
            return bool(res.success)
        return False

    def _call_set_mode(self, mode: str) -> bool:
        if not self.mode_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service /mavros/set_mode indisponible')
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            res = future.result()
            return bool(res.mode_sent)
        return False

    def _publish_target(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.active_target = msg  # pour diffusion continue
        self.setpoint_pub.publish(msg)

    def _heartbeat(self):
        if self.active_target is not None:
            # republie pour garder le contrôle offboard
            self.active_target.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.active_target)

    # ---------- API haut niveau (appelable via ros2 run + ros2 param plus tard) ----------
    def arm(self) -> bool:
        ok = self._call_arm(True)
        self.get_logger().info(f'arm(): {ok}')
        return ok

    def disarm(self) -> bool:
        ok = self._call_arm(False)
        self.get_logger().info(f'disarm(): {ok}')
        return ok

    def set_mode(self, mode: str) -> bool:
        ok = self._call_set_mode(mode)
        self.get_logger().info(f'set_mode({mode}): {ok}')
        return ok

    def takeoff_alt(self, alt: float = 5.0) -> bool:
        # GUIDED + arm + publier setpoint z
        ok = self.set_mode('GUIDED')
        ok = ok and self.arm()
        if not ok:
            return False

        # Publie des setpoints à 10 Hz pendant quelques secondes pour monter à l'altitude souhaitée
        t0 = self.get_clock().now()
        duration = Duration(seconds=8.0)
        self._publish_target(0.0, 0.0, alt)
        while self.get_clock().now() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Décollage vers {alt} m demandé')
        return True

    def goto_local(self, x: float, y: float, z: float) -> None:
        # suppose déjà GUIDED + arm. Publie en continu via _heartbeat
        self._publish_target(x, y, z)
        self.get_logger().info(f'Aller à (x={x}, y={y}, z={z})')

def main():
    rclpy.init()
    node = DroneInterface()
    # Petit “REPL” via paramètres (simple pour tester vite fait)
    # Exemples:
    #   ros2 param set /drone_interface command "TAKEOFF"
    #   ros2 param set /drone_interface args "[10.0]"
    node.declare_parameter('command', '')
    node.declare_parameter('args', [])
    node.declare_parameter('mode', '')  # Paramètre séparé pour les modes

    def on_param(event, node=node):
        cmd = node.get_parameter('command').get_parameter_value().string_value.upper()
        args_param = node.get_parameter('args')
        mode_param = node.get_parameter('mode').get_parameter_value().string_value
        
        args = []
        if args_param.type_ == rclpy.parameter.Parameter.Type.DOUBLE_ARRAY:
            args = [a for a in args_param.value]
        
        print(f"Received command: {cmd} with args: {args}, mode: {mode_param}")
        
        if cmd == 'ARM':
            print("Arming drone...")
            node.arm()
        elif cmd == 'DISARM':
            print("Disarming drone...")
            node.disarm()
        elif cmd == 'MODE':
            if mode_param:
                print(f"Setting mode to {mode_param}")
                node.set_mode(mode_param)
            else:
                print("MODE command requires mode parameter")
        elif cmd == 'TAKEOFF':
            alt = args[0] if args else 5.0
            print(f"Taking off to {alt} m")
            node.takeoff_alt(float(alt))
        elif cmd == 'GOTO':
            x, y, z = (args + [0.0, 0.0, 5.0])[:3]
            print(f"Going to (x={x}, y={y}, z={z})")
            node.goto_local(float(x), float(y), float(z))
        else:
            if cmd:
                print(f"Unknown command: {cmd}")
        
        # Ne pas réinitialiser ici pour éviter la boucle infinie
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(on_param)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
