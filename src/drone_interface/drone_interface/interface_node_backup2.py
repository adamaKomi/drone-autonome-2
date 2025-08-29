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
        # Initialise le nœud ROS2 avec le nom 'drone_interface'
        super().__init__('drone_interface')

        # Log d'information pour confirmer l'initialisation
        self.get_logger().info('Initialisation du noeud drone_interface...')

        # Configuration de la qualité de service (QoS) pour les topics
        # depth=10 signifie que 10 messages peuvent être mis en file d'attente
        qos = QoSProfile(depth=10)
        
        # Création d'un publisher pour envoyer des positions cibles au drone
        # PoseStamped contient position (x,y,z) + orientation + timestamp
        self.setpoint_pub = self.create_publisher(PoseStamped, 'drone/setpoint', qos)

        # Clients MAVROS pour communiquer avec l'autopilote
        # Client pour armer/désarmer le drone via MAVROS
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        # Client pour changer le mode de vol via MAVROS  
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Log de confirmation que l'initialisation est terminée
        self.get_logger().info('Noeud drone_interface initialisé avec succès!')

    # ---------- Méthode de diagnostic ----------
    def check_mavros_connection(self):
        """
        Vérifie si MAVROS est bien connecté en testant les services
        """
        self.get_logger().info('Vérification de la connexion MAVROS...')
        
        # Vérifier le service d'armement
        if self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('✅ Service /mavros/cmd/arming disponible')
        else:
            self.get_logger().error('❌ Service /mavros/cmd/arming non disponible')
            return False
            
        # Vérifier le service de mode
        if self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('✅ Service /mavros/set_mode disponible')
        else:
            self.get_logger().error('❌ Service /mavros/set_mode non disponible')
            return False
            
        self.get_logger().info('✅ MAVROS est bien connecté!')
        return True

    # ---------- Méthodes pour gérer les modes de vol ----------
    def set_mode(self, mode_name):
        """
        Change le mode de vol du drone
        mode_name: string - nom du mode (ex: 'GUIDED', 'STABILIZE', 'LOITER', etc.)
        Retourne True si succès, False sinon
        """
        # Vérifier que le service MAVROS est disponible
        if not self.mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Service /mavros/set_mode non disponible!')
            return False
        
        # Créer la requête pour changer le mode
        request = SetMode.Request()
        request.custom_mode = mode_name  # Mode personnalisé ArduPilot
        
        # Log de l'action en cours
        self.get_logger().info(f'Tentative de changement vers le mode: {mode_name}')
        
        try:
            # Envoyer la requête de manière asynchrone
            future = self.mode_client.call_async(request)
            
            # Attendre la réponse
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            # Vérifier si la requête s'est bien terminée
            if future.done():
                # Récupérer la réponse
                response = future.result()
                if response is not None and response.mode_sent:
                    # Succès du changement de mode
                    self.get_logger().info(f'✅ Mode changé vers: {mode_name}')
                    return True
                else:
                    # Échec du changement de mode
                    self.get_logger().error(f'❌ Échec du changement vers le mode: {mode_name}')
                    return False
            else:
                # Timeout
                self.get_logger().warn(f'Timeout lors du changement de mode vers: {mode_name}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Erreur lors du changement de mode: {str(e)}')
            return False

    # ---------- Méthodes pour envoyer des setpoints ----------
    def send_setpoint(self, x=0.0, y=0.0, z=2.0):
        """
        Envoie un setpoint de position au drone
        x, y, z: coordonnées en mètres dans le repère local
        """
        # Créer le message PoseStamped
        msg = PoseStamped()
        
        # En-tête avec timestamp actuel
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Repère de référence
        
        # Position cible
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y) 
        msg.pose.position.z = float(z)
        
        # Orientation (quaternion) - orientation neutre
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        
        # Publier le setpoint vers MAVROS
        self.setpoint_pub.publish(msg)
        
        # Log de l'action
        self.get_logger().info(f'Setpoint envoyé: x={x:.1f}, y={y:.1f}, z={z:.1f}m')

    def arm_and_hold_position(self, hold_altitude=2.0):
        """
        Arme le drone et maintient une position de hovering
        hold_altitude: altitude à maintenir en mètres
        """
        self.get_logger().info('=== ARMEMENT AVEC MAINTIEN DE POSITION ===')
        
        # Étape 1: Passer en mode GUIDED
        self.get_logger().info('Étape 1/3: Changement vers le mode GUIDED')
        if not self.set_mode('GUIDED'):
            self.get_logger().error('❌ Impossible de changer le mode')
            return False
        
        # Étape 2: Commencer à envoyer des setpoints AVANT l'armement
        self.get_logger().info(f'Étape 2/3: Envoi de setpoints à {hold_altitude}m')
        for i in range(10):  # Envoyer 10 setpoints avant armement
            self.send_setpoint(0.0, 0.0, hold_altitude)
            time.sleep(0.1)  # 10 Hz
            
        # Étape 3: Armer le drone
        self.get_logger().info('Étape 3/3: Armement du drone')
        success = self.arm_drone()
        
        if success:
            self.get_logger().info('✅ DRONE ARMÉ ET EN POSITION DE MAINTIEN!')
            self.get_logger().info(f'💡 Continuez à envoyer des setpoints pour maintenir le hovering')
            self.get_logger().info('💡 Utilisez: ros2 param set /drone_interface action "SETPOINT"')
            return True
        else:
            self.get_logger().error('❌ ÉCHEC DE L\'ARMEMENT')
            return False
        """
        Arme le drone en s'assurant qu'il est dans le bon mode
        target_mode: string - mode à utiliser avant l'armement (défaut: GUIDED)
        Retourne True si succès, False sinon
        """
        self.get_logger().info('=== SÉQUENCE D\'ARMEMENT AVEC VÉRIFICATION DU MODE ===')
        
        # Étape 1: Changer vers le mode cible
        self.get_logger().info(f'Étape 1/2: Changement vers le mode {target_mode}')
        if not self.set_mode(target_mode):
            self.get_logger().error('❌ Impossible de changer le mode, arrêt de la séquence')
            return False
        
        # Attendre un peu pour que le changement de mode soit effectif
        self.get_logger().info('Attente de 2 secondes pour stabiliser le mode...')
        time.sleep(2.0)
        
        # Étape 2: Armer le drone
        self.get_logger().info('Étape 2/2: Armement du drone')
        success = self.arm_drone()
        
        if success:
            self.get_logger().info('✅ SÉQUENCE D\'ARMEMENT TERMINÉE AVEC SUCCÈS!')
            self.get_logger().info('💡 Le drone devrait rester armé en mode GUIDED')
            self.get_logger().info('💡 Vous pouvez maintenant envoyer des setpoints de position')
        else:
            self.get_logger().error('❌ ÉCHEC DE LA SÉQUENCE D\'ARMEMENT')
            
        return success
    def arm_drone(self):
        """
        Arme le drone en envoyant une requête au service MAVROS
        Retourne True si succès, False sinon
        """
        # Vérifier que le service MAVROS est disponible avant d'essayer
        if not self.arm_client.wait_for_service(timeout_sec=3.0):
            # Si le service n'est pas disponible après 3 secondes
            self.get_logger().error('Service /mavros/cmd/arming non disponible!')
            return False
        
        # Créer la requête pour armer le drone
        request = CommandBool.Request()
        request.value = True  # True = armer, False = désarmer
        
        # Log de l'action en cours
        self.get_logger().info('Tentative d\'armement du drone...')
        
        try:
            # Envoyer la requête de manière asynchrone
            future = self.arm_client.call_async(request)
            
            # Attendre la réponse avec un timeout plus court
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            
            # Vérifier si la requête s'est bien terminée
            if future.done():
                # Récupérer la réponse
                response = future.result()
                if response is not None and response.success:
                    # Succès de l'armement
                    self.get_logger().info('Drone armé avec succès!')
                    self.get_logger().warn('⚠️  SITL peut se désarmer automatiquement après quelques secondes sans setpoints')
                    return True
                else:
                    # Échec de l'armement
                    self.get_logger().error('Échec de l\'armement du drone')
                    return False
            else:
                # Timeout - mais l'action peut avoir fonctionné côté SITL
                self.get_logger().warn('Timeout du service MAVROS, mais vérifiez les logs SITL')
                self.get_logger().info('L\'armement peut avoir réussi même avec ce timeout')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Erreur lors de l\'armement: {str(e)}')
            return False

    def disarm_drone(self):
        """
        Désarme le drone en envoyant une requête au service MAVROS
        Retourne True si succès, False sinon
        """
        # Vérifier que le service MAVROS est disponible
        if not self.arm_client.wait_for_service(timeout_sec=3.0):
            # Si le service n'est pas disponible après 3 secondes
            self.get_logger().error('Service /mavros/cmd/arming non disponible!')
            return False
        
        # Créer la requête pour désarmer le drone
        request = CommandBool.Request()
        request.value = False  # False = désarmer
        
        # Log de l'action en cours
        self.get_logger().info('Tentative de désarmement du drone...')
        
        try:
            # Envoyer la requête de manière asynchrone
            future = self.arm_client.call_async(request)
            
            # Attendre la réponse avec un timeout plus court
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            
            # Vérifier si la requête s'est bien terminée
            if future.done():
                # Récupérer la réponse
                response = future.result()
                if response is not None and response.success:
                    # Succès du désarmement
                    self.get_logger().info('Drone désarmé avec succès!')
                    return True
                else:
                    # Échec du désarmement
                    self.get_logger().error('Échec du désarmement du drone')
                    return False
            else:
                # Timeout - mais l'action peut avoir fonctionné côté SITL
                self.get_logger().warn('Timeout du service MAVROS, mais vérifiez les logs SITL')
                self.get_logger().info('Le désarmement peut avoir réussi même avec ce timeout')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Erreur lors du désarmement: {str(e)}')
            return False

















def main():
    # Initialiser ROS2
    rclpy.init()
    
    # Créer une instance du nœud drone_interface
    node = DroneInterface()
    
    # Déclarer des paramètres pour contrôler le drone via la ligne de commande
    # Paramètre pour spécifier l'action à effectuer
    node.declare_parameter('action', '')  # 'ARM', 'DISARM', 'CHECK', 'MODE', 'ARM_GUIDED'
    node.declare_parameter('mode', '')    # Mode cible pour la commande MODE
    
    def on_parameter_change(parameters):
        """
        Callback appelé quand un paramètre change
        Permet de contrôler le drone via: ros2 param set /drone_interface action "ARM"
        """
        for parameter in parameters:
            # Vérifier si le paramètre 'action' a été modifié
            if parameter.name == 'action':
                # Récupérer la valeur du paramètre en majuscules
                action = parameter.value.upper()
                
                # Récupérer le mode cible si spécifié
                mode_param = node.get_parameter('mode').get_parameter_value().string_value.upper()
                
                # Log de l'action reçue
                node.get_logger().info(f'Action reçue: {action}' + (f' (mode: {mode_param})' if mode_param else ''))
                
                # Exécuter l'action correspondante
                if action == 'ARM':
                    # Armer le drone (simple)
                    success = node.arm_drone()
                    if success:
                        node.get_logger().info('✅ Armement réussi!')
                    else:
                        node.get_logger().error('❌ Armement échoué!')
                        
                elif action == 'ARM_GUIDED':
                    # Armer le drone avec changement automatique vers GUIDED
                    success = node.arm_drone_with_mode_check('GUIDED')
                    if success:
                        node.get_logger().info('✅ Armement en mode GUIDED réussi!')
                    else:
                        node.get_logger().error('❌ Armement en mode GUIDED échoué!')
                        
                elif action == 'ARM_HOLD':
                    # Armer le drone avec maintien de position automatique
                    success = node.arm_and_hold_position(2.0)  # 2 mètres d'altitude
                    if success:
                        node.get_logger().info('✅ Armement avec maintien de position réussi!')
                    else:
                        node.get_logger().error('❌ Armement avec maintien de position échoué!')
                        
                elif action == 'SETPOINT':
                    # Envoyer un setpoint de position (pour maintien de position)
                    node.send_setpoint(0.0, 0.0, 2.0)  # Position de hovering
                    node.get_logger().info('✅ Setpoint de maintien envoyé!')
                        
                elif action == 'DISARM':
                    # Désarmer le drone
                    success = node.disarm_drone()
                    if success:
                        node.get_logger().info('✅ Désarmement réussi!')
                    else:
                        node.get_logger().error('❌ Désarmement échoué!')
                        
                elif action == 'MODE':
                    # Changer de mode
                    if mode_param:
                        success = node.set_mode(mode_param)
                        if success:
                            node.get_logger().info(f'✅ Mode changé vers {mode_param}!')
                        else:
                            node.get_logger().error(f'❌ Échec du changement vers {mode_param}!')
                    else:
                        node.get_logger().error('❌ Paramètre mode requis! Utilisez: ros2 param set /drone_interface mode "GUIDED"')
                        
                elif action == 'CHECK':
                    # Vérifier la connexion MAVROS
                    node.check_mavros_connection()
                        
                elif action != '':
                    # Action non reconnue
                    node.get_logger().warn(f'Action non reconnue: {action}')
                    node.get_logger().info('Actions disponibles: ARM, ARM_GUIDED, ARM_HOLD, SETPOINT, DISARM, MODE, CHECK')
                
                # Remettre le paramètre à vide pour éviter les répétitions
                if action != '':
                    node.set_parameters([rclpy.parameter.Parameter('action', rclpy.Parameter.Type.STRING, '')])
        
        # Retourner le résultat de l'opération
        return SetParametersResult(successful=True)
    
    # Enregistrer le callback pour les changements de paramètres
    node.add_on_set_parameters_callback(on_parameter_change)
    
    # Log d'instructions pour l'utilisateur
    node.get_logger().info('=== INTERFACE DE TEST ARMER/DÉSARMER AVANCÉE ===')
    node.get_logger().info('Commandes disponibles:')
    node.get_logger().info('  ARM        : ros2 param set /drone_interface action "ARM"')
    node.get_logger().info('  ARM_GUIDED : ros2 param set /drone_interface action "ARM_GUIDED"')
    node.get_logger().info('  ARM_HOLD   : ros2 param set /drone_interface action "ARM_HOLD"')
    node.get_logger().info('  SETPOINT   : ros2 param set /drone_interface action "SETPOINT"')
    node.get_logger().info('  DISARM     : ros2 param set /drone_interface action "DISARM"')
    node.get_logger().info('  MODE       : ros2 param set /drone_interface mode "GUIDED" && ros2 param set /drone_interface action "MODE"')
    node.get_logger().info('  CHECK      : ros2 param set /drone_interface action "CHECK"')
    node.get_logger().info('=====================================')
    node.get_logger().info('💡 Recommandé: Utilisez ARM_HOLD pour un armement stable avec hovering')
    node.get_logger().info('💡 Puis SETPOINT pour maintenir la position')
    
    try:
        # Démarrer la boucle principale du nœud
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gestion propre de l'arrêt avec Ctrl+C
        node.get_logger().info('Arrêt du nœud drone_interface...')
    finally:
        # Nettoyage des ressources
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()