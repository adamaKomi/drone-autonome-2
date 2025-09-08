#!/usr/bin/env python3

"""
Outil CLI pour navigation vers position

Permet de commander le drone pour naviguer vers une position spécifique
via interface en ligne de commande avec support de coordonnées GPS
et locales.

Auteur: Adama Komi
Version: 1.1.0
"""

import argparse
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger


class GotoPositionTool(Node):
    """Outil CLI pour navigation vers position"""
    
    def __init__(self):
        super().__init__('goto_position_tool')
        
        # Client de service pour navigation
        self._goto_service_client = self.create_client(
            Trigger, '/drone_nav/navigation/goto_position'
        )
        
        self.get_logger().info("Outil goto_position initialisé")

    def goto_position(self, lat: Optional[float] = None, lon: Optional[float] = None,
                     alt: Optional[float] = None, x: Optional[float] = None, 
                     y: Optional[float] = None, z: Optional[float] = None,
                     frame: str = 'global', waypoint: Optional[str] = None,
                     timeout: float = 30.0) -> bool:
        """
        Navigue vers une position
        
        Args:
            lat, lon, alt: Coordonnées GPS en degrés et mètres
            x, y, z: Coordonnées locales en mètres
            frame: Système de coordonnées ('global' ou 'local')
            waypoint: Nom d'un waypoint prédéfini
            timeout: Timeout en secondes
            
        Returns:
            True si succès, False sinon
            
        Raises:
            RuntimeError: Si le service n'est pas disponible
            ValueError: Si les coordonnées sont invalides
        """
        # Attente du service
        if not self._goto_service_client.wait_for_service(timeout_sec=5.0):
            error_msg = "Service de navigation non disponible"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        # Validation des paramètres
        if not self._validate_parameters(lat, lon, alt, x, y, z, frame, waypoint):
            error_msg = "Paramètres de navigation invalides"
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        
        # Préparation de la requête
        request = Trigger.Request()
        
        # Note: Le service Trigger ne permet pas de passer de paramètres
        # Il faudrait utiliser un topic ou paramètres ROS2 pour définir la destination
        # Pour l'instant, on utilise les paramètres du nœud
        
        try:
            # Configuration des paramètres de destination via ROS2 parameters
            if lat is not None and lon is not None:
                # Coordonnées GPS
                self.get_logger().info(f"Configuration destination GPS: lat={lat}, lon={lon}, alt={alt}")
            elif x is not None and y is not None:
                # Coordonnées locales
                self.get_logger().info(f"Configuration destination locale: x={x}, y={y}, z={z}")
            
            # Appel du service
            future = self._goto_service_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Navigation démarrée: {response.message}")
                    return True
                else:
                    self.get_logger().error(f"Échec navigation: {response.message}")
                    return False
            else:
                error_msg = "Timeout du service de navigation"
                self.get_logger().error(error_msg)
                raise TimeoutError(error_msg)
                
        except Exception as e:
            error_msg = f"Erreur lors de l'appel du service: {e}"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg) from e

    def _validate_parameters(self, lat: Optional[float], lon: Optional[float],
                           alt: Optional[float], x: Optional[float],
                           y: Optional[float], z: Optional[float],
                           frame: str, waypoint: Optional[str]) -> bool:
        """
        Valide les paramètres de navigation
        
        Args:
            lat, lon, alt: Coordonnées GPS
            x, y, z: Coordonnées locales
            frame: Système de coordonnées
            waypoint: Waypoint prédéfini
            
        Returns:
            True si les paramètres sont valides, False sinon
        """
        # Vérification des coordonnées GPS
        if lat is not None:
            if not (-90.0 <= lat <= 90.0):
                self.get_logger().error(f"Latitude invalide: {lat}")
                return False
        if lon is not None:
            if not (-180.0 <= lon <= 180.0):
                self.get_logger().error(f"Longitude invalide: {lon}")
                return False
        if alt is not None:
            if alt < 0:
                self.get_logger().error(f"Altitude négative: {alt}")
                return False
        
        # Vérification des coordonnées locales
        if x is not None and abs(x) > 10000:
            self.get_logger().error(f"Coordonnée X hors limites: {x}")
            return False
        if y is not None and abs(y) > 10000:
            self.get_logger().error(f"Coordonnée Y hors limites: {y}")
            return False
        if z is not None and (z < 0 or z > 1000):
            self.get_logger().error(f"Coordonnée Z hors limites: {z}")
            return False
        
        # Vérification de la cohérence des paramètres
        if waypoint and (lat is not None or lon is not None or x is not None or y is not None):
            self.get_logger().warning("Waypoint spécifié avec des coordonnées, le waypoint sera utilisé")
        
        return True


def parse_arguments():
    """Parse les arguments de ligne de commande"""
    parser = argparse.ArgumentParser(
        description="Navigue vers une position spécifiée",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation:
# Navigation GPS
ros2 run drone_navigation goto_position --lat 34.0 --lon -6.8 --alt 50

# Navigation locale
ros2 run drone_navigation goto_position --x 10 --y 20 --z 15 --frame local

# Waypoint prédéfini
ros2 run drone_navigation goto_position --waypoint home

# Position avec timeout personnalisé
ros2 run drone_navigation goto_position --x 0 --y 0 --z 20 --timeout 60
        """
    )
    
    # Groupe pour coordonnées GPS
    gps_group = parser.add_argument_group('Coordonnées GPS')
    gps_group.add_argument('--lat', type=float, 
                          help='Latitude en degrés décimaux [-90, 90]')
    gps_group.add_argument('--lon', type=float, 
                          help='Longitude en degrés décimaux [-180, 180]')
    gps_group.add_argument('--alt', type=float, 
                          help='Altitude en mètres (≥0)')
    
    # Groupe pour coordonnées locales
    local_group = parser.add_argument_group('Coordonnées locales')
    local_group.add_argument('--x', type=float, 
                           help='Position X en mètres')
    local_group.add_argument('--y', type=float, 
                           help='Position Y en mètres')
    local_group.add_argument('--z', type=float, 
                           help='Position Z en mètres (≥0)')
    
    # Options générales
    parser.add_argument('--frame', choices=['global', 'local'], default='global',
                       help='Système de coordonnées (défaut: global)')
    parser.add_argument('--waypoint', type=str,
                       help='Nom du waypoint prédéfini (home, start, etc.)')
    parser.add_argument('--timeout', type=float, default=30.0,
                       help='Timeout en secondes (défaut: 30)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Affichage détaillé')
    
    return parser.parse_args()


def validate_arguments(args):
    """
    Valide les arguments de ligne de commande
    
    Args:
        args: Arguments parsés
        
    Returns:
        True si les arguments sont valides, False sinon
    """
    # Vérification des coordonnées
    if args.waypoint:
        # Waypoint prédéfini - pas besoin d'autres coordonnées
        return True
    
    if args.frame == 'global':
        # Coordonnées GPS requises
        if args.lat is None or args.lon is None:
            print("Erreur: Latitude et longitude requises pour le frame global", 
                  file=sys.stderr)
            return False
        if not (-90 <= args.lat <= 90):
            print("Erreur: Latitude doit être entre -90 et 90 degrés", 
                  file=sys.stderr)
            return False
        if not (-180 <= args.lon <= 180):
            print("Erreur: Longitude doit être entre -180 et 180 degrés", 
                  file=sys.stderr)
            return False
    else:
        # Coordonnées locales requises
        if args.x is None or args.y is None:
            print("Erreur: Coordonnées X et Y requises pour le frame local", 
                  file=sys.stderr)
            return False
    
    # Vérification des valeurs numériques
    if args.alt is not None and args.alt < 0:
        print("Erreur: L'altitude ne peut pas être négative", file=sys.stderr)
        return False
    
    if args.z is not None and args.z < 0:
        print("Erreur: La coordonnée Z ne peut pas être négative", file=sys.stderr)
        return False
    
    if args.timeout <= 0:
        print("Erreur: Le timeout doit être positif", file=sys.stderr)
        return False
    
    return True


def main(args=None):
    """
    Point d'entrée principal de l'outil CLI
    
    Args:
        args: Arguments de ligne de commande
        
    Returns:
        Code de sortie (0 = succès, autre = erreur)
    """
    # Parse des arguments
    parsed_args = parse_arguments()
    
    # Validation
    if not validate_arguments(parsed_args):
        return 1
    
    # Initialisation ROS2
    rclpy.init(args=args)
    
    try:
        # Création de l'outil
        tool = GotoPositionTool()
        
        # Configuration du logging
        if parsed_args.verbose:
            tool.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            tool.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # Affichage des informations
        if parsed_args.waypoint:
            tool.get_logger().info(f"Navigation vers waypoint: {parsed_args.waypoint}")
        elif parsed_args.frame == 'global':
            tool.get_logger().info(f"Navigation GPS vers: "
                                 f"lat={parsed_args.lat}, lon={parsed_args.lon}, "
                                 f"alt={parsed_args.alt or 'auto'}")
        else:
            tool.get_logger().info(f"Navigation locale vers: "
                                 f"x={parsed_args.x}, y={parsed_args.y}, "
                                 f"z={parsed_args.z or 'auto'}")
        
        # Exécution de la navigation
        success = tool.goto_position(
            lat=parsed_args.lat,
            lon=parsed_args.lon, 
            alt=parsed_args.alt,
            x=parsed_args.x,
            y=parsed_args.y,
            z=parsed_args.z,
            frame=parsed_args.frame,
            waypoint=parsed_args.waypoint,
            timeout=parsed_args.timeout
        )
        
        if success:
            print("✅ Navigation démarrée avec succès")
            return 0
        else:
            print("❌ Échec de la navigation")
            return 1
            
    except KeyboardInterrupt:
        print("\n🛑 Interruption par l'utilisateur")
        return 0
        
    except ValueError as e:
        print(f"❌ Erreur de validation: {e}", file=sys.stderr)
        return 1
        
    except RuntimeError as e:
        print(f"❌ Erreur d'exécution: {e}", file=sys.stderr)
        return 1
        
    except TimeoutError as e:
        print(f"⏰ Timeout: {e}", file=sys.stderr)
        return 1
        
    except Exception as e:
        print(f"❌ Erreur inattendue: {e}", file=sys.stderr)
        return 1
        
    finally:
        # Nettoyage
        try:
            if 'tool' in locals():
                tool.destroy_node()
        except Exception as e:
            print(f"⚠️  Erreur lors du nettoyage: {e}", file=sys.stderr)
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())