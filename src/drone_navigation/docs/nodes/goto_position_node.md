# Documentation complète : `goto_position_node`

## 1. Présentation

Le nœud `goto_position_node` gère la navigation autonome du drone vers une position GPS ou locale. Il intègre des vérifications de sécurité, la gestion dynamique de la cible, le feedback en temps réel, et la compatibilité avec MAVROS/ArduPilot.

## 2. Fonctionnalités principales
- Navigation vers une position GPS ou locale
- Vérifications de sécurité (connexion MAVROS, armement, mode, altitude, batterie)
- Mise à jour dynamique de la cible en vol
- Annulation et maintien de position
- Feedback détaillé (progression, distance, statut)
- Publication continue des setpoints

## 3. Interfaces ROS2

### Services
- `/drone_nav/goto_position` : Navigation vers une position GPS
- `/drone_nav/goto_local` : Navigation vers une position locale
- `/drone_nav/update_position` : Mise à jour de la cible GPS en vol
- `/drone_nav/update_local` : Mise à jour de la cible locale en vol

### Action
- `/drone_nav/goto_position_action` : Navigation GPS avec feedback et annulation

### Topics publiés
- `/drone_nav/path_progress` : Progression de la navigation (`drone_msgs/msg/PathProgress`)
- `/drone_nav/status` : Statut de la navigation (`drone_msgs/msg/NavigationStatus`)

## 4. Utilisation

### Exemples de commandes
```bash
# Navigation GPS
ros2 service call /drone_nav/goto_position drone_msgs/srv/GotoPosition "{latitude: 48.8584, longitude: 2.2945, altitude: 10.0, tolerance: 1.0}"

# Navigation locale
ros2 service call /drone_nav/goto_local drone_msgs/srv/GotoLocal "{x: 5.0, y: 3.0, z: 2.0, yaw_angle: 0.0}"

# Mise à jour de la cible locale en vol
ros2 service call /drone_nav/update_local drone_msgs/srv/GotoLocal "{x: 10.0, y: 5.0, z: 10.0, yaw_angle: 0.0}"

# Action avec feedback
ros2 action send_goal /drone_nav/goto_position_action drone_msgs/action/GotoPositionAction "{latitude: 48.8584, longitude: 2.2945, altitude: 10.0, tolerance: 1.0}"
```

### Lancement du nœud
```bash
ros2 launch drone_navigation global_drone_nodes.launch.py
```

## 5. Sécurité et vérifications
- Le nœud vérifie la connexion MAVROS, l'armement, le mode de vol, l'altitude minimale, et le niveau de batterie avant toute navigation.
- En cas de batterie faible ou d'annulation, le drone maintient sa position actuelle.

## 6. Mise à jour dynamique de la cible

### Mise à jour de la cible GPS
- Utilisez le service `/drone_nav/update_position` pour modifier la cible GPS pendant une navigation active.
- Exemple :
```bash
ros2 service call /drone_nav/update_position drone_msgs/srv/GotoPosition "{latitude: 48.8590, longitude: 2.2950, altitude: 12.0, tolerance: 1.0}"
```
- La nouvelle cible est prise en compte immédiatement, le drone ajuste sa trajectoire et le feedback est mis à jour sur les topics.

### Mise à jour de la cible locale
- Utilisez `/drone_nav/update_local` pour modifier la cible locale pendant une navigation active.
- Exemple :
```bash
ros2 service call /drone_nav/update_local drone_msgs/srv/GotoLocal "{x: 10.0, y: 5.0, z: 10.0, yaw_angle: 0.0}"
```
- La transition est appliquée immédiatement et le feedback est mis à jour.

## 7. Monitoring et feedback
- Suivez la progression sur `/drone_nav/path_progress`.
- Suivez le statut sur `/drone_nav/status`.
- Les logs du nœud fournissent des informations détaillées sur l'état et les erreurs éventuelles.

## 8. Tests recommandés

### Pré-requis
- MAVROS connecté au simulateur ou au drone réel
- Drone armé et en mode GUIDED/AUTO/LOITER/POSCTL

### Scénarios de test
1. **Test navigation GPS**
   - Envoyer une commande `/drone_nav/goto_position` avec des coordonnées valides
   - Vérifier la progression et le statut
2. **Test navigation locale**
   - Envoyer une commande `/drone_nav/goto_local` avec des coordonnées locales
   - Vérifier la progression et le statut
3. **Test mise à jour de cible**
   - Pendant une navigation, envoyer `/drone_nav/update_local` ou `/drone_nav/update_position` avec une nouvelle cible
   - Vérifier que le drone change de cible et que le feedback est mis à jour
4. **Test annulation**
   - Annuler une action en cours (via l'action ou interruption)
   - Vérifier que le drone maintient sa position
5. **Test sécurité**
   - Simuler une batterie faible ou une altitude insuffisante
   - Vérifier que la navigation est refusée ou interrompue

## 9. Bonnes pratiques
- Toujours vérifier le statut du drone avant d'envoyer une commande
- Surveiller les topics de feedback pour le suivi en temps réel
- Utiliser les services de mise à jour pour adapter la mission en vol

## 10. Référence code
Le code source complet est disponible dans :
`src/drone_navigation/drone_navigation/nodes/goto_position_node.py`

---

Pour toute question ou amélioration, consulter la documentation du package ou le code source.
