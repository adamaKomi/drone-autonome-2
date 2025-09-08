#!/usr/bin/env python3

"""
Script de calibration du système de navigation

Effectue la calibration automatique des capteurs, contrôleurs PID
et systèmes de navigation pour optimiser les performances.

Auteur: Adama Komi
Version: 1.1.0
"""

import argparse
import time
import json
import sys
import math
import numpy as np
from typing import Dict, List, Optional, Tuple, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, Vector3, Quaternion
from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSet
from geographic_msgs.msg import GeoPoint


class NavigationCalibrator(Node):
    """Calibrateur automatique du système de navigation"""
    
    def __init__(self):
        super().__init__('navigation_calibrator')
        
        self.calibration_results = {}
        self.calibration_data = []
        self.mavros_state = None
        self.current_position = None
        self.current_velocity = None
        self.imu_data = None
        self.mag_data = None
        self.extended_state = None
        
        # Configuration QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)
        self.position_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', 
            self.position_callback, qos_profile)
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, qos_profile)
        self.mag_sub = self.create_subscription(
            MagneticField, '/mavros/imu/mag', self.mag_callback, qos_profile)
        self.extended_state_sub = self.create_subscription(
            ExtendedState, '/mavros/extended_state', 
            self.extended_state_callback, qos_profile)
            
        # Services clients
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_service = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_service = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.param_service = self.create_client(ParamSet, '/mavros/param/set')
        
        # Attendre que les services soient disponibles
        self.get_logger().info("En attente des services MAVROS...")
        self.arm_service.wait_for_service(timeout_sec=10.0)
        self.mode_service.wait_for_service(timeout_sec=10.0)
        
        # Publisher pour statut de calibration
        self.status_pub = self.create_publisher(String, '/drone_nav/calibration_status', 10)
        
        self.get_logger().info("Navigation Calibrator initialized")
        
    def state_callback(self, msg: State):
        """Callback pour état MAVROS"""
        self.mavros_state = msg
        
    def position_callback(self, msg: NavSatFix):
        """Callback pour position GPS"""
        self.current_position = msg
        
    def imu_callback(self, msg: Imu):
        """Callback pour données IMU"""
        self.imu_data = msg
        
    def mag_callback(self, msg: MagneticField):
        """Callback pour données magnétomètre"""
        self.mag_data = msg
        
    def extended_state_callback(self, msg: ExtendedState):
        """Callback pour état étendu"""
        self.extended_state = msg
        
    def wait_for_connections(self, timeout: float = 30.0) -> bool:
        """Attendre les connexions nécessaires
        
        Args:
            timeout: Délai maximum d'attente en secondes
            
        Returns:
            bool: True si toutes les connexions sont établies, False sinon
        """
        self.get_logger().info("En attente des connexions MAVROS...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if (self.mavros_state is not None and 
                self.current_position is not None and
                self.imu_data is not None and
                self.mag_data is not None):
                self.get_logger().info("Toutes les connexions sont établies")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.get_logger().error("Timeout lors de l'attente des connexions")
        return False
        
    def set_flight_mode(self, mode: str) -> bool:
        """Changer le mode de vol du drone
        
        Args:
            mode: Mode de vol (ex: 'GUIDED', 'LAND', 'RTL')
            
        Returns:
            bool: True si le changement de mode a réussi
        """
        try:
            req = SetMode.Request()
            req.custom_mode = mode
            
            future = self.mode_service.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() and future.result().mode_sent:
                self.get_logger().info(f"Mode changé vers: {mode}")
                return True
            else:
                self.get_logger().error(f"Échec du changement de mode vers: {mode}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Erreur lors du changement de mode: {e}")
            return False
            
    def arm_drone(self, arm: bool = True) -> bool:
        """Armer ou désarmer le drone
        
        Args:
            arm: True pour armer, False pour désarmer
            
        Returns:
            bool: True si l'opération a réussi
        """
        try:
            req = CommandBool.Request()
            req.value = arm
            
            future = self.arm_service.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() and future.result().success:
                action = "armé" if arm else "désarmé"
                self.get_logger().info(f"Drone {action} avec succès")
                return True
            else:
                action = "armer" if arm else "désarmer"
                self.get_logger().error(f"Échec pour {action} le drone")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'armement: {e}")
            return False
        
    def calibrate_compass(self) -> Dict[str, Any]:
        """Calibration de la boussole
        
        Returns:
            Dict: Résultats de la calibration de la boussole
        """
        self.get_logger().info("Démarrage de la calibration de la boussole...")
        
        # Données de calibration boussole
        compass_data = []
        calibration_time = 30.0  # 30 secondes
        
        self.get_logger().info("Tournez le drone lentement dans toutes les directions pendant 30 secondes...")
        
        start_time = time.time()
        while time.time() - start_time < calibration_time:
            if self.mag_data:
                # Enregistrer données magnétomètre
                mag_data = {
                    'x': self.mag_data.magnetic_field.x,
                    'y': self.mag_data.magnetic_field.y,
                    'z': self.mag_data.magnetic_field.z,
                    'timestamp': time.time()
                }
                compass_data.append(mag_data)
                
            rclpy.spin_once(self, timeout_sec=0.1)
            
        # Analyser données de calibration
        if len(compass_data) > 100:
            # Calculer offsets et facteurs d'échelle
            x_values = [d['x'] for d in compass_data]
            y_values = [d['y'] for d in compass_data]
            z_values = [d['z'] for d in compass_data]
            
            # Méthode de calibration simple (ellipsoid fitting simplifié)
            x_offset = (max(x_values) + min(x_values)) / 2
            y_offset = (max(y_values) + min(y_values)) / 2
            z_offset = (max(z_values) + min(z_values)) / 2
            
            x_scale = (max(x_values) - min(x_values)) / 2
            y_scale = (max(y_values) - min(y_values)) / 2
            z_scale = (max(z_values) - min(z_values)) / 2
            
            avg_scale = (x_scale + y_scale + z_scale) / 3
            
            compass_result = {
                'success': True,
                'offsets': {'x': x_offset, 'y': y_offset, 'z': z_offset},
                'scales': {'x': avg_scale/x_scale, 'y': avg_scale/y_scale, 'z': avg_scale/z_scale},
                'data_points': len(compass_data),
                'quality': 'excellent' if len(compass_data) > 250 else 'good'
            }
        else:
            compass_result = {
                'success': False,
                'error': 'Données insuffisantes',
                'data_points': len(compass_data)
            }
            
        self.get_logger().info(f"Calibration de la boussole terminée: {compass_result['success']}")
        return compass_result
        
    def calibrate_accelerometer(self, auto_mode: bool = False) -> Dict[str, Any]:
        """Calibration de l'accéléromètre
        
        Args:
            auto_mode: Si True, utilise des données en vol plutôt qu'une interaction utilisateur
            
        Returns:
            Dict: Résultats de la calibration de l'accéléromètre
        """
        self.get_logger().info("Démarrage de la calibration de l'accéléromètre...")
        
        if auto_mode:
            return self._calibrate_accelerometer_auto()
        else:
            return self._calibrate_accelerometer_manual()
            
    def _calibrate_accelerometer_manual(self) -> Dict[str, Any]:
        """Calibration manuelle de l'accéléromètre (nécessite interaction utilisateur)"""
        self.get_logger().warn("Mode manuel non supporté dans ROS - utilisation du mode automatique")
        return self._calibrate_accelerometer_auto()
        
    def _calibrate_accelerometer_auto(self) -> Dict[str, Any]:
        """Calibration automatique de l'accéléromètre en utilisant les données en vol"""
        self.get_logger().info("Calibration automatique de l'accéléromètre...")
        
        # Collecter des données pendant le vol stable
        accel_samples = []
        collection_time = 20.0  # 20 secondes de données
        
        self.get_logger().info("Collecte des données d'accéléromètre en vol...")
        
        start_time = time.time()
        while time.time() - start_time < collection_time:
            if self.imu_data:
                accel_samples.append({
                    'x': self.imu_data.linear_acceleration.x,
                    'y': self.imu_data.linear_acceleration.y,
                    'z': self.imu_data.linear_acceleration.z,
                    'timestamp': time.time()
                })
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if len(accel_samples) > 100:
            # Calculer les offsets (devrait être ~0,0,-9.8 en conditions statiques)
            x_vals = [s['x'] for s in accel_samples]
            y_vals = [s['y'] for s in accel_samples]
            z_vals = [s['z'] for s in accel_samples]
            
            x_offset = np.mean(x_vals)
            y_offset = np.mean(y_vals)
            z_offset = np.mean(z_vals) + 9.8  # Compensation gravité
            
            # Calculer le bruit/écart-type
            x_noise = np.std(x_vals)
            y_noise = np.std(y_vals)
            z_noise = np.std(z_vals)
            
            accel_result = {
                'success': True,
                'offsets': {'x': x_offset, 'y': y_offset, 'z': z_offset},
                'noise_levels': {'x': x_noise, 'y': y_noise, 'z': z_noise},
                'samples': len(accel_samples),
                'quality': 'excellent' if max(x_noise, y_noise, z_noise) < 0.1 else 'good'
            }
        else:
            accel_result = {
                'success': False,
                'error': 'Données insuffisantes pour la calibration',
                'samples': len(accel_samples)
            }
            
        self.get_logger().info(f"Calibration de l'accéléromètre terminée: {accel_result['success']}")
        return accel_result
        
    def _calculate_distance_meters(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calcule la distance en mètres entre deux points GPS
        
        Args:
            lat1, lon1: Première coordonnée
            lat2, lon2: Deuxième coordonnée
            
        Returns:
            float: Distance en mètres
        """
        # Formule de Haversine
        R = 6371000  # Rayon de la Terre en mètres
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_phi/2) * math.sin(delta_phi/2) +
             math.cos(phi1) * math.cos(phi2) *
             math.sin(delta_lambda/2) * math.sin(delta_lambda/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
        
    def calibrate_gps_position(self) -> Dict[str, Any]:
        """Calibration de la position GPS
        
        Returns:
            Dict: Résultats de la calibration GPS
        """
        self.get_logger().info("Démarrage de la calibration GPS...")
        
        if not self.current_position:
            return {'success': False, 'error': 'Aucune donnée GPS disponible'}
            
        # Collecter données GPS pendant 60 secondes
        gps_samples = []
        calibration_time = 60.0
        
        self.get_logger().info("Collecte des données GPS pendant 60 secondes (drone stationnaire)...")
        
        start_time = time.time()
        while time.time() - start_time < calibration_time:
            if self.current_position and self.current_position.status.status >= 0:  # Fix valide
                gps_samples.append({
                    'lat': self.current_position.latitude,
                    'lon': self.current_position.longitude,
                    'alt': self.current_position.altitude,
                    'eph': self.current_position.position_covariance[0] if self.current_position.position_covariance else 0,
                    'epv': self.current_position.position_covariance[4] if self.current_position.position_covariance else 0,
                    'timestamp': time.time()
                })
            rclpy.spin_once(self, timeout_sec=1.0)
            
        if len(gps_samples) > 30:
            # Analyser précision GPS
            lats = [s['lat'] for s in gps_samples]
            lons = [s['lon'] for s in gps_samples]
            alts = [s['alt'] for s in gps_samples]
            ephs = [s['eph'] for s in gps_samples]
            epvs = [s['epv'] for s in gps_samples]
            
            # Position moyenne comme référence
            ref_position = {
                'latitude': np.mean(lats),
                'longitude': np.mean(lons),
                'altitude': np.mean(alts)
            }
            
            # Calculer la déviation standard en mètres
            lat_ref = ref_position['latitude']
            lon_ref = ref_position['longitude']
            
            # Convertir les déviations en mètres
            lat_errors = [self._calculate_distance_meters(lat, lon_ref, lat_ref, lon_ref) 
                         for lat in lats]
            lon_errors = [self._calculate_distance_meters(lat_ref, lon, lat_ref, lon_ref) 
                         for lon in lons]
            
            horizontal_std = max(np.std(lat_errors), np.std(lon_errors))
            vertical_std = np.std(alts)
            
            # Précision estimée
            horizontal_accuracy = np.mean(ephs) if any(ephs) else horizontal_std * 2
            vertical_accuracy = np.mean(epvs) if any(epvs) else vertical_std * 2
            
            # Évaluer qualité
            if horizontal_accuracy < 1.0 and vertical_accuracy < 2.0:
                quality = 'excellent'
            elif horizontal_accuracy < 3.0 and vertical_accuracy < 5.0:
                quality = 'good'
            elif horizontal_accuracy < 5.0 and vertical_accuracy < 10.0:
                quality = 'fair'
            else:
                quality = 'poor'
                
            gps_result = {
                'success': True,
                'reference_position': ref_position,
                'horizontal_accuracy_m': float(horizontal_accuracy),
                'vertical_accuracy_m': float(vertical_accuracy),
                'hdop': float(np.mean(ephs)),
                'vdop': float(np.mean(epvs)),
                'quality': quality,
                'samples': len(gps_samples)
            }
        else:
            gps_result = {
                'success': False,
                'error': 'Échantillons GPS insuffisants',
                'samples': len(gps_samples)
            }
            
        self.get_logger().info(f"Calibration GPS terminée: {gps_result['success']}")
        return gps_result
        
    def calibrate_flight_controller(self) -> Dict[str, Any]:
        """Calibration des paramètres de contrôle de vol
        
        Returns:
            Dict: Résultats de la calibration du contrôleur de vol
        """
        self.get_logger().info("Démarrage de la calibration du contrôleur de vol...")
        
        if not self.mavros_state or not self.mavros_state.connected:
            return {'success': False, 'error': 'MAVROS non connecté'}
            
        # Vérifier que le drone est armé et en mode approprié
        if not self.mavros_state.armed:
            self.get_logger().warn("Le drone doit être armé pour la calibration du contrôleur")
            return {'success': False, 'error': 'Drone non armé'}
            
        # Test de stabilité en hover
        self.get_logger().info("Test de stabilité en vol stationnaire (20 secondes)...")
        
        position_samples = []
        test_duration = 20.0
        start_time = time.time()
        
        while time.time() - start_time < test_duration:
            if self.current_position and self.current_position.status.status >= 0:
                position_samples.append({
                    'lat': self.current_position.latitude,
                    'lon': self.current_position.longitude,
                    'alt': self.current_position.altitude,
                    'timestamp': time.time()
                })
            rclpy.spin_once(self, timeout_sec=0.2)
            
        if len(position_samples) > 50:
            # Analyser stabilité
            lats = [s['lat'] for s in position_samples]
            lons = [s['lon'] for s in position_samples]
            alts = [s['alt'] for s in position_samples]
            
            # Convertir en déviations en mètres
            lat_ref = np.mean(lats)
            lon_ref = np.mean(lons)
            alt_ref = np.mean(alts)
            
            lat_errors = [self._calculate_distance_meters(lat, lon_ref, lat_ref, lon_ref) 
                         for lat in lats]
            lon_errors = [self._calculate_distance_meters(lat_ref, lon, lat_ref, lon_ref) 
                         for lon in lons]
            
            max_horizontal_drift = max(max(lat_errors), max(lon_errors))
            max_vertical_drift = max(alts) - min(alts)
            
            # Évaluer la stabilité
            if max_horizontal_drift < 0.5 and max_vertical_drift < 1.0:
                stability_quality = 'excellent'
            elif max_horizontal_drift < 1.0 and max_vertical_drift < 2.0:
                stability_quality = 'good'
            elif max_horizontal_drift < 2.0 and max_vertical_drift < 3.0:
                stability_quality = 'fair'
            else:
                stability_quality = 'poor'
                
            controller_result = {
                'success': True,
                'hover_stability': {
                    'max_horizontal_drift_m': float(max_horizontal_drift),
                    'max_vertical_drift_m': float(max_vertical_drift),
                    'quality': stability_quality
                },
                'test_duration': test_duration,
                'samples': len(position_samples)
            }
        else:
            controller_result = {
                'success': False,
                'error': 'Données de position insuffisantes pour le test du contrôleur',
                'samples': len(position_samples)
            }
            
        self.get_logger().info(f"Calibration du contrôleur terminée: {controller_result['success']}")
        return controller_result
        
    def generate_calibration_report(self) -> str:
        """Générer rapport de calibration complet
        
        Returns:
            str: Rapport de calibration formaté en JSON
        """
        report = {
            'calibration_timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'results': self.calibration_results,
            'overall_quality': self._assess_overall_quality(),
            'recommendations': self._generate_recommendations(),
            'system_info': {
                'mavros_connected': self.mavros_state.connected if self.mavros_state else False,
                'gps_fix': self.current_position.status.status if self.current_position else -1,
                'timestamp': time.time()
            }
        }
        
        return json.dumps(report, indent=2)
        
    def _assess_overall_quality(self) -> str:
        """Évaluer la qualité globale de la calibration
        
        Returns:
            str: Qualité globale ('excellent', 'good', 'fair', 'poor')
        """
        successful_calibrations = sum(1 for result in self.calibration_results.values() 
                                    if result.get('success', False))
        total_calibrations = len(self.calibration_results)
        
        if successful_calibrations == total_calibrations:
            # Vérifier la qualité individuelle
            qualities = [result.get('quality', 'poor') for result in self.calibration_results.values()
                        if result.get('success', False)]
            if all(q == 'excellent' for q in qualities):
                return 'excellent'
            elif all(q in ['excellent', 'good'] for q in qualities):
                return 'good'
            else:
                return 'fair'
        elif successful_calibrations >= total_calibrations * 0.75:
            return 'good'
        elif successful_calibrations >= total_calibrations * 0.5:
            return 'fair'
        else:
            return 'poor'
            
    def _generate_recommendations(self) -> List[str]:
        """Générer recommandations basées sur les résultats
        
        Returns:
            List[str]: Liste des recommandations
        """
        recommendations = []
        
        for cal_type, result in self.calibration_results.items():
            if not result.get('success', False):
                recommendations.append(f"Réessayer la calibration {cal_type}")
            elif result.get('quality') in ['fair', 'poor']:
                recommendations.append(f"Améliorer la qualité de calibration {cal_type}")
                
        if not recommendations:
            recommendations.append("Toutes les calibrations sont réussies - système prêt pour le vol")
            
        return recommendations
        
    def run_full_calibration(self, auto_mode: bool = False) -> bool:
        """Exécuter calibration complète
        
        Args:
            auto_mode: Si True, utilise le mode automatique sans interaction utilisateur
            
        Returns:
            bool: True si la calibration a réussi
        """
        self.get_logger().info("Démarrage de la calibration complète du système de navigation")
        
        if not self.wait_for_connections():
            return False
            
        # Calibrations dans l'ordre recommandé
        calibrations = [
            ('compass', lambda: self.calibrate_compass()),
            ('accelerometer', lambda: self.calibrate_accelerometer(auto_mode)),
            ('gps', lambda: self.calibrate_gps_position()),
            ('controller', lambda: self.calibrate_flight_controller())
        ]
        
        for cal_name, cal_func in calibrations:
            self.get_logger().info(f"\n--- Démarrage de la calibration {cal_name} ---")
            try:
                result = cal_func()
                self.calibration_results[cal_name] = result
                
                # Publier le statut
                status_msg = String()
                status_msg.data = json.dumps({
                    'calibration': cal_name,
                    'result': result,
                    'timestamp': time.time()
                })
                self.status_pub.publish(status_msg)
                
                if not result.get('success', False):
                    self.get_logger().error(f"Calibration {cal_name} échouée: {result.get('error', 'Unknown error')}")
                else:
                    self.get_logger().info(f"Calibration {cal_name} réussie")
                    
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la calibration {cal_name}: {e}")
                self.calibration_results[cal_name] = {
                    'success': False,
                    'error': str(e)
                }
                
        # Générer rapport final
        report = self.generate_calibration_report()
        self.get_logger().info("\n--- Rapport de Calibration ---")
        print(report)
        
        # Sauvegarder rapport
        timestamp = int(time.time())
        filename = f"calibration_report_{timestamp}.json"
        with open(filename, 'w') as f:
            f.write(report)
            
        self.get_logger().info(f"Rapport sauvegardé: {filename}")
        
        # Évaluer le succès global
        successful = sum(1 for r in self.calibration_results.values() if r.get('success', False))
        return successful >= len(self.calibration_results) * 0.5  # Au moins 50% de réussite


def main():
    """Point d'entrée principal"""
    parser = argparse.ArgumentParser(description='Calibrateur du Système de Navigation')
    parser.add_argument('--calibration-type', choices=['all', 'compass', 'accel', 'gps', 'controller'],
                       default='all', help='Type de calibration à effectuer')
    parser.add_argument('--output-file', help='Fichier pour sauvegarder les résultats')
    parser.add_argument('--auto-mode', action='store_true', 
                       help='Exécuter en mode automatique (sans interaction utilisateur)')
    parser.add_argument('--timeout', type=float, default=30.0,
                       help='Timeout pour les connexions (secondes)')
    
    args = parser.parse_args()
    
    rclpy.init()
    calibrator = NavigationCalibrator()
    
    success = False
    try:
        if args.calibration_type == 'all':
            success = calibrator.run_full_calibration(args.auto_mode)
        else:
            # Calibration spécifique
            calibration_methods = {
                'compass': calibrator.calibrate_compass,
                'accel': lambda: calibrator.calibrate_accelerometer(args.auto_mode),
                'gps': calibrator.calibrate_gps_position,
                'controller': calibrator.calibrate_flight_controller
            }
            
            if args.calibration_type in calibration_methods:
                result = calibration_methods[args.calibration_type]()
                calibrator.calibration_results[args.calibration_type] = result
                success = result.get('success', False)
                
                print(json.dumps(result, indent=2))
            else:
                calibrator.get_logger().error(f"Type de calibration inconnu: {args.calibration_type}")
                success = False
                
        if args.output_file and calibrator.calibration_results:
            report = calibrator.generate_calibration_report()
            with open(args.output_file, 'w') as f:
                f.write(report)
            calibrator.get_logger().info(f"Résultats sauvegardés dans: {args.output_file}")
            
    except KeyboardInterrupt:
        calibrator.get_logger().info("Calibration interrompue par l'utilisateur")
        success = False
    except Exception as e:
        calibrator.get_logger().error(f"Erreur lors de la calibration: {e}")
        success = False
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()
        
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()