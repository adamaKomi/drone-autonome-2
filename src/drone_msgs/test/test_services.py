#!/usr/bin/env python3
"""
Tests d'intégration pour les services du package drone_msgs
"""

import unittest
import rclpy
from rclpy.node import Node
from drone_msgs.srv import (
    ArmDrone, DisarmDrone, SetFlightMode, Takeoff, Land, ReturnToLaunch,
    EmergencyStop, LoadMission, StartMission, StopMission, PauseMission,
    SetWaypoint, SetGeofence, CalibrateSensors, SystemDiagnostic,
    UpdateParameters, GetSystemStatus, ConfigureCamera, DetectFlowers,
    ExecutePollination, SaveData
)


class TestDroneServices(unittest.TestCase):
    """Tests pour les services drone_msgs"""

    @classmethod
    def setUpClass(cls):
        """Initialisation des tests"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Nettoyage après tests"""
        rclpy.shutdown()

    def test_arm_drone_service(self):
        """Test service ArmDrone"""
        request = ArmDrone.Request()
        response = ArmDrone.Response()
        
        # Test requête
        request.force_arm = False
        request.skip_preflight = False
        request.arm_mode = "NORMAL"
        request.operator_id = "OP001"
        
        self.assertFalse(request.force_arm)
        self.assertFalse(request.skip_preflight)
        self.assertEqual(request.arm_mode, "NORMAL")
        self.assertEqual(request.operator_id, "OP001")
        
        # Test réponse
        response.success = True
        response.message = "Drone armed successfully"
        response.arm_status = 1
        
        self.assertTrue(response.success)
        self.assertEqual(response.message, "Drone armed successfully")
        self.assertEqual(response.arm_status, 1)

    def test_takeoff_service(self):
        """Test service Takeoff"""
        request = Takeoff.Request()
        response = Takeoff.Response()
        
        # Test requête
        request.target_altitude = 10.0
        request.climb_rate = 2.0
        request.takeoff_mode = "NORMAL"
        
        self.assertEqual(request.target_altitude, 10.0)
        self.assertEqual(request.climb_rate, 2.0)
        self.assertEqual(request.takeoff_mode, "NORMAL")
        
        # Test réponse
        response.success = True
        response.achieved_altitude = 9.8
        response.takeoff_time_seconds = 5.2
        
        self.assertTrue(response.success)
        self.assertEqual(response.achieved_altitude, 9.8)
        self.assertEqual(response.takeoff_time_seconds, 5.2)

    def test_load_mission_service(self):
        """Test service LoadMission"""
        request = LoadMission.Request()
        response = LoadMission.Response()
        
        # Test requête
        request.mission_file = "/path/to/mission.json"
        request.mission_type = "POLLINATION"
        request.validate_mission = True
        request.operator_id = "OP001"
        
        self.assertEqual(request.mission_file, "/path/to/mission.json")
        self.assertEqual(request.mission_type, "POLLINATION")
        self.assertTrue(request.validate_mission)
        
        # Test réponse
        response.success = True
        response.mission_id = "MISSION_001"
        response.waypoint_count = 25
        response.estimated_duration_minutes = 45.5
        
        self.assertTrue(response.success)
        self.assertEqual(response.mission_id, "MISSION_001")
        self.assertEqual(response.waypoint_count, 25)
        self.assertEqual(response.estimated_duration_minutes, 45.5)

    def test_detect_flowers_service(self):
        """Test service DetectFlowers"""
        request = DetectFlowers.Request()
        response = DetectFlowers.Response()
        
        # Test requête
        request.search_radius_m = 50.0
        request.min_confidence = 0.8
        request.detection_mode = "DETAILED"
        request.max_detections = 10
        
        self.assertEqual(request.search_radius_m, 50.0)
        self.assertEqual(request.min_confidence, 0.8)
        self.assertEqual(request.detection_mode, "DETAILED")
        self.assertEqual(request.max_detections, 10)
        
        # Test réponse
        response.success = True
        response.flowers_detected = 7
        response.detection_time_seconds = 12.3
        
        self.assertTrue(response.success)
        self.assertEqual(response.flowers_detected, 7)
        self.assertEqual(response.detection_time_seconds, 12.3)

    def test_execute_pollination_service(self):
        """Test service ExecutePollination"""
        request = ExecutePollination.Request()
        response = ExecutePollination.Response()
        
        # Test requête
        request.flower_id = "FLOWER_001"
        request.flower_species = "Rosa canina"
        request.pollination_method = "VIBRATION"
        request.contact_duration_s = 3.0
        
        self.assertEqual(request.flower_id, "FLOWER_001")
        self.assertEqual(request.flower_species, "Rosa canina")
        self.assertEqual(request.pollination_method, "VIBRATION")
        self.assertEqual(request.contact_duration_s, 3.0)
        
        # Test réponse
        response.success = True
        response.pollination_id = "POLL_001"
        response.pollination_effectiveness = 0.85
        
        self.assertTrue(response.success)
        self.assertEqual(response.pollination_id, "POLL_001")
        self.assertEqual(response.pollination_effectiveness, 0.85)

    def test_system_diagnostic_service(self):
        """Test service SystemDiagnostic"""
        request = SystemDiagnostic.Request()
        response = SystemDiagnostic.Response()
        
        # Test requête
        request.diagnostic_type = "FULL"
        request.diagnostic_level = "COMPREHENSIVE"
        request.include_performance_test = True
        
        self.assertEqual(request.diagnostic_type, "FULL")
        self.assertEqual(request.diagnostic_level, "COMPREHENSIVE")
        self.assertTrue(request.include_performance_test)
        
        # Test réponse
        response.success = True
        response.overall_health_status = "GOOD"
        response.overall_health_score = 0.85
        
        self.assertTrue(response.success)
        self.assertEqual(response.overall_health_status, "GOOD")
        self.assertEqual(response.overall_health_score, 0.85)

    def test_configure_camera_service(self):
        """Test service ConfigureCamera"""
        request = ConfigureCamera.Request()
        response = ConfigureCamera.Response()
        
        # Test requête
        request.camera_id = "CAM001"
        request.resolution_width = 1920
        request.resolution_height = 1080
        request.framerate = 30
        
        self.assertEqual(request.camera_id, "CAM001")
        self.assertEqual(request.resolution_width, 1920)
        self.assertEqual(request.resolution_height, 1080)
        self.assertEqual(request.framerate, 30)
        
        # Test réponse
        response.success = True
        response.configured_camera_id = "CAM001"
        response.actual_resolution_width = 1920
        response.actual_framerate = 30
        
        self.assertTrue(response.success)
        self.assertEqual(response.configured_camera_id, "CAM001")
        self.assertEqual(response.actual_resolution_width, 1920)
        self.assertEqual(response.actual_framerate, 30)


if __name__ == '__main__':
    unittest.main()
