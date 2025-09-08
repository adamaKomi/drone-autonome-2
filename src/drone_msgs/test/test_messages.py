#!/usr/bin/env python3
"""
Tests unitaires pour les messages du package drone_msgs
"""

import unittest
import rclpy
from rclpy.node import Node
from drone_msgs.msg import (
    DroneStatus, DroneState, FlightMode, SafetyStatus, BatteryStatus,
    Position3D, Orientation, Velocity3D, NavigationCommand, Waypoint,
    Trajectory, FlowerDetection, FlowerClassification, FlowerTarget,
    PollinationResult, MissionStatus, MissionProgress, MissionWaypoint,
    ZoneDefinition, EnvironmentData, PerformanceMetrics, SystemAlert,
    DiagnosticInfo
)


class TestDroneMsgs(unittest.TestCase):
    """Tests pour les messages drone_msgs"""

    @classmethod
    def setUpClass(cls):
        """Initialisation des tests"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Nettoyage après tests"""
        rclpy.shutdown()

    def test_drone_status_creation(self):
        """Test création message DroneStatus"""
        msg = DroneStatus()
        self.assertIsNotNone(msg)
        
        # Test des champs requis
        msg.drone_id = "test_drone_001"
        msg.flight_mode = "MANUAL"
        msg.is_armed = True
        msg.battery_percentage = 85.5
        
        self.assertEqual(msg.drone_id, "test_drone_001")
        self.assertEqual(msg.flight_mode, "MANUAL")
        self.assertTrue(msg.is_armed)
        self.assertEqual(msg.battery_percentage, 85.5)

    def test_drone_state_creation(self):
        """Test création message DroneState"""
        msg = DroneState()
        self.assertIsNotNone(msg)
        
        msg.system_id = "SYS_001"
        msg.autopilot_type = "PIXHAWK"
        msg.system_status = "STANDBY"
        
        self.assertEqual(msg.system_id, "SYS_001")
        self.assertEqual(msg.autopilot_type, "PIXHAWK")
        self.assertEqual(msg.system_status, "STANDBY")

    def test_flight_mode_creation(self):
        """Test création message FlightMode"""
        msg = FlightMode()
        self.assertIsNotNone(msg)
        
        msg.mode_name = "AUTO"
        msg.mode_number = 4
        msg.can_arm = True
        msg.allows_takeoff = True
        
        self.assertEqual(msg.mode_name, "AUTO")
        self.assertEqual(msg.mode_number, 4)
        self.assertTrue(msg.can_arm)
        self.assertTrue(msg.allows_takeoff)

    def test_safety_status_creation(self):
        """Test création message SafetyStatus"""
        msg = SafetyStatus()
        self.assertIsNotNone(msg)
        
        msg.overall_safety_level = "SAFE"
        msg.preflight_checks_passed = True
        msg.geofence_status = "INSIDE"
        
        self.assertEqual(msg.overall_safety_level, "SAFE")
        self.assertTrue(msg.preflight_checks_passed)
        self.assertEqual(msg.geofence_status, "INSIDE")

    def test_battery_status_creation(self):
        """Test création message BatteryStatus"""
        msg = BatteryStatus()
        self.assertIsNotNone(msg)
        
        msg.voltage = 12.6
        msg.current = 2.5
        msg.percentage = 75.0
        msg.temperature = 25.5
        
        self.assertEqual(msg.voltage, 12.6)
        self.assertEqual(msg.current, 2.5)
        self.assertEqual(msg.percentage, 75.0)
        self.assertEqual(msg.temperature, 25.5)

    def test_position_3d_creation(self):
        """Test création message Position3D"""
        msg = Position3D()
        self.assertIsNotNone(msg)
        
        msg.latitude = 45.123456
        msg.longitude = 2.654321
        msg.altitude_msl = 150.5
        
        self.assertEqual(msg.latitude, 45.123456)
        self.assertEqual(msg.longitude, 2.654321)
        self.assertEqual(msg.altitude_msl, 150.5)

    def test_orientation_creation(self):
        """Test création message Orientation"""
        msg = Orientation()
        self.assertIsNotNone(msg)
        
        msg.roll = 0.1
        msg.pitch = -0.05
        msg.yaw = 1.57
        
        self.assertEqual(msg.roll, 0.1)
        self.assertEqual(msg.pitch, -0.05)
        self.assertEqual(msg.yaw, 1.57)

    def test_velocity_3d_creation(self):
        """Test création message Velocity3D"""
        msg = Velocity3D()
        self.assertIsNotNone(msg)
        
        msg.vx = 2.0
        msg.vy = 1.5
        msg.vz = -0.5
        
        self.assertEqual(msg.vx, 2.0)
        self.assertEqual(msg.vy, 1.5)
        self.assertEqual(msg.vz, -0.5)

    def test_navigation_command_creation(self):
        """Test création message NavigationCommand"""
        msg = NavigationCommand()
        self.assertIsNotNone(msg)
        
        msg.command_type = "GOTO"
        msg.priority = 5
        msg.timeout_seconds = 30.0
        
        self.assertEqual(msg.command_type, "GOTO")
        self.assertEqual(msg.priority, 5)
        self.assertEqual(msg.timeout_seconds, 30.0)

    def test_waypoint_creation(self):
        """Test création message Waypoint"""
        msg = Waypoint()
        self.assertIsNotNone(msg)
        
        msg.waypoint_id = "WP_001"
        msg.sequence_number = 1
        msg.waypoint_type = "NORMAL"
        
        self.assertEqual(msg.waypoint_id, "WP_001")
        self.assertEqual(msg.sequence_number, 1)
        self.assertEqual(msg.waypoint_type, "NORMAL")

    def test_flower_detection_creation(self):
        """Test création message FlowerDetection"""
        msg = FlowerDetection()
        self.assertIsNotNone(msg)
        
        msg.detection_id = "FLOWER_001"
        msg.confidence = 0.95
        msg.species_detected = "Rosa canina"
        
        self.assertEqual(msg.detection_id, "FLOWER_001")
        self.assertEqual(msg.confidence, 0.95)
        self.assertEqual(msg.species_detected, "Rosa canina")

    def test_pollination_result_creation(self):
        """Test création message PollinationResult"""
        msg = PollinationResult()
        self.assertIsNotNone(msg)
        
        msg.operation_id = "POLL_001"
        msg.success = True
        msg.effectiveness_score = 0.85
        
        self.assertEqual(msg.operation_id, "POLL_001")
        self.assertTrue(msg.success)
        self.assertEqual(msg.effectiveness_score, 0.85)

    def test_mission_status_creation(self):
        """Test création message MissionStatus"""
        msg = MissionStatus()
        self.assertIsNotNone(msg)
        
        msg.mission_id = "MISSION_001"
        msg.status = "ACTIVE"
        msg.progress_percentage = 45.5
        
        self.assertEqual(msg.mission_id, "MISSION_001")
        self.assertEqual(msg.status, "ACTIVE")
        self.assertEqual(msg.progress_percentage, 45.5)

    def test_environment_data_creation(self):
        """Test création message EnvironmentData"""
        msg = EnvironmentData()
        self.assertIsNotNone(msg)
        
        msg.temperature = 22.5
        msg.humidity = 65.0
        msg.wind_speed = 3.2
        
        self.assertEqual(msg.temperature, 22.5)
        self.assertEqual(msg.humidity, 65.0)
        self.assertEqual(msg.wind_speed, 3.2)

    def test_system_alert_creation(self):
        """Test création message SystemAlert"""
        msg = SystemAlert()
        self.assertIsNotNone(msg)
        
        msg.alert_id = "ALERT_001"
        msg.alert_type = "WARNING"
        msg.priority_level = 3
        
        self.assertEqual(msg.alert_id, "ALERT_001")
        self.assertEqual(msg.alert_type, "WARNING")
        self.assertEqual(msg.priority_level, 3)


if __name__ == '__main__':
    unittest.main()
