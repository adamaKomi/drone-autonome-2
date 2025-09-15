# emergency_handler_node.md

## Description générale
Le nœud `emergency_handler_node` est responsable de la gestion des situations d'urgence pour le drone autonome. Il surveille en temps réel l'état du drone (batterie, GPS, communications MAVROS, armement) et déclenche automatiquement les procédures d'urgence appropriées en cas d'anomalie critique. Il publie le statut d'urgence et la sécurité pour les autres nœuds, protège les actions contre le spam, et assure la coordination avec MAVROS pour les changements de mode (LAND, RTL, LOITER). Ce nœud est conçu pour garantir la sécurité et la robustesse du drone autonome en mission.

## Fonctionnalités principales
- Surveillance continue de la batterie, du signal GPS, de la connexion MAVROS et de l'état d'armement.
- Détection et priorisation des situations d'urgence : batterie faible/critique, perte GPS, perte de communication, etc.
- Déclenchement automatique des procédures d'urgence : atterrissage immédiat, retour à la maison, maintien de position, attente de reconnexion.
- Publication du statut d'urgence et du signal de sécurité pour les autres nœuds ROS2.
- Protection contre le spam d'actions d'urgence (cooldown configurable).
- Interface thread-safe pour la gestion des états et des publications.

## Topics et Services
- **Souscriptions** :
  - `/mavros/state` (State) : état MAVROS (connexion, armement, mode)
  - `/mavros/battery` (BatteryState) : niveau de batterie
  - `/mavros/global_position/global` (NavSatFix) : position GPS
  - `/mavros/local_position/pose` (PoseStamped) : position locale
- **Publications** :
  - `/drone_nav/emergency_status` (String) : statut d'urgence courant
  - `/drone_nav/emergency_trigger` (Bool) : signal d'urgence actif
  - `/drone_nav/safe_to_navigate` (Bool) : signal de sécurité pour navigation
- **Services clients** :
  - `/mavros/cmd/arming` (CommandBool) : armement/désarmement du drone
  - `/mavros/set_mode` (SetMode) : changement de mode de vol

## Paramètres configurables
- `low_battery_threshold` : seuil batterie faible (%)
- `critical_battery_threshold` : seuil batterie critique (%)
- `gps_timeout` : délai max sans signal GPS (s)
- `comms_timeout` : délai max sans communication MAVROS (s)
- `return_home_altitude` : altitude de retour à la maison (m)
- `emergency_action_cooldown` : délai de protection contre le spam d'actions (s)

## Logique d'urgence
- Priorité des urgences : CRITICAL > GPS_LOSS > LOW_BATTERY > COMMUNICATION_LOSS > NORMAL
- Déclenchement d'une procédure uniquement si la nouvelle urgence est plus prioritaire que l'actuelle.
- Procédures associées :
  - **CRITICAL** : atterrissage immédiat (LAND)
  - **GPS_LOSS** : maintien de position (LOITER)
  - **LOW_BATTERY** : retour à la maison (RTL)
  - **COMMUNICATION_LOSS** : attente de reconnexion

## Sécurité et robustesse
- Toutes les opérations critiques sont protégées par des verrous (thread-safe).
- Les publications vers les autres nœuds sont également thread-safe.
- Le nœud assure la réinitialisation automatique de l'état d'urgence dès que la situation redevient normale.

## Exemple d'utilisation
Ce nœud doit être lancé en parallèle des autres nœuds de navigation et de mission. Il assure la sécurité globale du drone et informe les autres composants du système en cas de problème.

---

## Plan de test complet

Pour tester entièrement toutes les fonctionnalités du nœud `emergency_handler_node`, il faut simuler chaque situation d'urgence et vérifier les réactions attendues (publications, changements de mode, logs, etc.).

### 1. Pré-requis
- Lancer le nœud avec ROS2 et MAVROS actifs (ou simuler MAVROS).
- Utiliser des outils comme `ros2 topic pub`, `ros2 service call`, et observer les logs.

### 2. Tests unitaires par fonctionnalité

#### a) Batterie faible et critique
- Publier un message `BatteryState` avec `percentage` < `low_battery_threshold` puis < `critical_battery_threshold`.
- Vérifier :
  - Publication sur `/drone_nav/emergency_status` et `/drone_nav/emergency_trigger`.
  - Passage en mode RTL (batterie faible) puis LAND (batterie critique) via `/mavros/set_mode`.
  - Log d’erreur approprié.

#### b) Perte du signal GPS
- Arrêter la publication de `NavSatFix` ou publier un message avec `status.status < 0` pendant plus de `gps_timeout` secondes.
- Vérifier :
  - Déclenchement de l’urgence GPS_LOSS.
  - Passage en mode LOITER.
  - Publication et logs.

#### c) Perte de communication MAVROS
- Arrêter la publication sur `/mavros/state` ou simuler une déconnexion.
- Attendre plus de `comms_timeout` secondes.
- Vérifier :
  - Déclenchement de l’urgence COMMUNICATION_LOSS.
  - Attente de reconnexion (pas de changement de mode).
  - Publication et logs.

#### d) Retour à la normale
- Rétablir la publication des messages manquants (batterie, GPS, MAVROS).
- Vérifier :
  - Réinitialisation de l’état d’urgence à NORMAL.
  - Publication sur `/drone_nav/emergency_status` et `/drone_nav/emergency_trigger` (désactivé).
  - Log d’information.

#### e) Protection contre le spam d’actions
- Déclencher plusieurs fois la même urgence en moins de `emergency_action_cooldown` secondes.
- Vérifier :
  - Une seule action d’urgence exécutée (pas de spam).

#### f) Sécurité de navigation
- Vérifier la publication régulière sur `/drone_nav/safe_to_navigate` selon l’état d’urgence et la connexion MAVROS.

### 3. Outils recommandés
- `ros2 topic pub` pour simuler les messages.
- `ros2 topic echo` pour observer les publications.
- `ros2 service call` pour tester les clients MAVROS.
- Observer les logs du nœud pour chaque test.

### 4. Automatisation
- Écrire un script de test Python ou bash qui enchaîne ces scénarios et vérifie les résultats automatiquement.

---

Auteur : adamaKomi
Date : 2025-09-14
