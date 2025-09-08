#!/usr/bin/env python3
"""
Tests unitaires pour les actions du package drone_msgs
"""

import unittest
import rclpy
from rclpy.node import Node
from drone_msgs.action import (
    ExecuteMission, NavigateToPosition, FollowTrajectory, SearchAndPollinate,
    MonitorZone, CollectData, MapArea, PerformMaintenance, EmergencyLanding
)


class TestDroneActions(unittest.TestCase):
    """Tests pour les actions drone_msgs"""

    @classmethod
    def setUpClass(cls):
        """Initialisation des tests"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Nettoyage après tests"""
        rclpy.shutdown()

    def test_execute_mission_action(self):
        """Test action ExecuteMission"""
        goal = ExecuteMission.Goal()
        result = ExecuteMission.Result()
        feedback = ExecuteMission.Feedback()
        
        # Test goal
        goal.mission_id = "MISSION_001"
        goal.auto_takeoff = True
        goal.execution_mode = "NORMAL"
        goal.speed_factor = 1.0
        
        self.assertEqual(goal.mission_id, "MISSION_001")
        self.assertTrue(goal.auto_takeoff)
        self.assertEqual(goal.execution_mode, "NORMAL")
        self.assertEqual(goal.speed_factor, 1.0)
        
        # Test result
        result.success = True
        result.completion_status = "COMPLETED"
        result.completion_percentage = 100.0
        result.flowers_pollinated = 25
        
        self.assertTrue(result.success)
        self.assertEqual(result.completion_status, "COMPLETED")
        self.assertEqual(result.completion_percentage, 100.0)
        self.assertEqual(result.flowers_pollinated, 25)
        
        # Test feedback
        feedback.progress_percentage = 50.0
        feedback.current_waypoint_index = 12
        feedback.current_task = "POLLINATING"
        
        self.assertEqual(feedback.progress_percentage, 50.0)
        self.assertEqual(feedback.current_waypoint_index, 12)
        self.assertEqual(feedback.current_task, "POLLINATING")

    def test_navigate_to_position_action(self):
        """Test action NavigateToPosition"""
        goal = NavigateToPosition.Goal()
        result = NavigateToPosition.Result()
        feedback = NavigateToPosition.Feedback()
        
        # Test goal
        goal.target_altitude = 15.0
        goal.speed_limit = 5.0
        goal.use_obstacle_avoidance = True
        goal.path_planning_mode = "SAFE"
        
        self.assertEqual(goal.target_altitude, 15.0)
        self.assertEqual(goal.speed_limit, 5.0)
        self.assertTrue(goal.use_obstacle_avoidance)
        self.assertEqual(goal.path_planning_mode, "SAFE")
        
        # Test result
        result.success = True
        result.completion_status = "REACHED"
        result.position_error_m = 0.5
        result.navigation_time_seconds = 45.2
        
        self.assertTrue(result.success)
        self.assertEqual(result.completion_status, "REACHED")
        self.assertEqual(result.position_error_m, 0.5)
        self.assertEqual(result.navigation_time_seconds, 45.2)
        
        # Test feedback
        feedback.distance_to_target_m = 25.3
        feedback.current_speed_ms = 3.2
        feedback.estimated_time_to_arrival_s = 8.0
        
        self.assertEqual(feedback.distance_to_target_m, 25.3)
        self.assertEqual(feedback.current_speed_ms, 3.2)
        self.assertEqual(feedback.estimated_time_to_arrival_s, 8.0)

    def test_search_and_pollinate_action(self):
        """Test action SearchAndPollinate"""
        goal = SearchAndPollinate.Goal()
        result = SearchAndPollinate.Result()
        feedback = SearchAndPollinate.Feedback()
        
        # Test goal
        goal.search_area_radius_m = 100.0
        goal.max_flowers_to_pollinate = 50
        goal.min_detection_confidence = 0.8
        goal.search_pattern = "SPIRAL"
        
        self.assertEqual(goal.search_area_radius_m, 100.0)
        self.assertEqual(goal.max_flowers_to_pollinate, 50)
        self.assertEqual(goal.min_detection_confidence, 0.8)
        self.assertEqual(goal.search_pattern, "SPIRAL")
        
        # Test result
        result.success = True
        result.completion_status = "COMPLETED"
        result.flowers_detected = 35
        result.flowers_pollinated = 32
        
        self.assertTrue(result.success)
        self.assertEqual(result.completion_status, "COMPLETED")
        self.assertEqual(result.flowers_detected, 35)
        self.assertEqual(result.flowers_pollinated, 32)
        
        # Test feedback
        feedback.current_phase = "POLLINATING"
        feedback.area_searched_m2 = 5000.0
        feedback.flowers_pollinated_so_far = 15
        
        self.assertEqual(feedback.current_phase, "POLLINATING")
        self.assertEqual(feedback.area_searched_m2, 5000.0)
        self.assertEqual(feedback.flowers_pollinated_so_far, 15)

    def test_monitor_zone_action(self):
        """Test action MonitorZone"""
        goal = MonitorZone.Goal()
        result = MonitorZone.Result()
        feedback = MonitorZone.Feedback()
        
        # Test goal
        goal.surveillance_altitude_m = 20.0
        goal.patrol_speed_ms = 2.0
        goal.patrol_pattern = "PERIMETER"
        goal.continuous_recording = True
        
        self.assertEqual(goal.surveillance_altitude_m, 20.0)
        self.assertEqual(goal.patrol_speed_ms, 2.0)
        self.assertEqual(goal.patrol_pattern, "PERIMETER")
        self.assertTrue(goal.continuous_recording)
        
        # Test result
        result.success = True
        result.completion_status = "COMPLETED"
        result.patrol_cycles_completed = 5
        result.detections_made = 12
        
        self.assertTrue(result.success)
        self.assertEqual(result.completion_status, "COMPLETED")
        self.assertEqual(result.patrol_cycles_completed, 5)
        self.assertEqual(result.detections_made, 12)
        
        # Test feedback
        feedback.current_activity = "PATROLLING"
        feedback.current_patrol_cycle = 3
        feedback.total_detections = 8
        
        self.assertEqual(feedback.current_activity, "PATROLLING")
        self.assertEqual(feedback.current_patrol_cycle, 3)
        self.assertEqual(feedback.total_detections, 8)

    def test_collect_data_action(self):
        """Test action CollectData"""
        goal = CollectData.Goal()
        result = CollectData.Result()
        feedback = CollectData.Feedback()
        
        # Test goal
        goal.collection_altitude_m = 12.0
        goal.collection_pattern = "SEQUENTIAL"
        goal.high_resolution_mode = True
        goal.real_time_processing = False
        
        self.assertEqual(goal.collection_altitude_m, 12.0)
        self.assertEqual(goal.collection_pattern, "SEQUENTIAL")
        self.assertTrue(goal.high_resolution_mode)
        self.assertFalse(goal.real_time_processing)
        
        # Test result
        result.success = True
        result.completion_status = "COMPLETED"
        result.points_visited = 25
        result.data_samples_collected = 250
        
        self.assertTrue(result.success)
        self.assertEqual(result.completion_status, "COMPLETED")
        self.assertEqual(result.points_visited, 25)
        self.assertEqual(result.data_samples_collected, 250)
        
        # Test feedback
        feedback.current_point_index = 15
        feedback.collection_progress_percentage = 60.0
        feedback.data_samples_collected_so_far = 150
        
        self.assertEqual(feedback.current_point_index, 15)
        self.assertEqual(feedback.collection_progress_percentage, 60.0)
        self.assertEqual(feedback.data_samples_collected_so_far, 150)

    def test_emergency_landing_action(self):
        """Test action EmergencyLanding"""
        goal = EmergencyLanding.Goal()
        result = EmergencyLanding.Result()
        feedback = EmergencyLanding.Feedback()
        
        # Test goal
        goal.emergency_type = "BATTERY_CRITICAL"
        goal.max_descent_rate = 3.0
        goal.use_emergency_power = True
        goal.broadcast_emergency = True
        
        self.assertEqual(goal.emergency_type, "BATTERY_CRITICAL")
        self.assertEqual(goal.max_descent_rate, 3.0)
        self.assertTrue(goal.use_emergency_power)
        self.assertTrue(goal.broadcast_emergency)
        
        # Test result
        result.success = True
        result.landing_status = "SAFE_LANDING"
        result.descent_time_seconds = 25.5
        result.drone_recoverable = True
        
        self.assertTrue(result.success)
        self.assertEqual(result.landing_status, "SAFE_LANDING")
        self.assertEqual(result.descent_time_seconds, 25.5)
        self.assertTrue(result.drone_recoverable)
        
        # Test feedback
        feedback.current_emergency_phase = "DESCENDING"
        feedback.current_altitude = 8.5
        feedback.descent_rate_ms = 2.5
        
        self.assertEqual(feedback.current_emergency_phase, "DESCENDING")
        self.assertEqual(feedback.current_altitude, 8.5)
        self.assertEqual(feedback.descent_rate_ms, 2.5)


if __name__ == '__main__':
    unittest.main()
