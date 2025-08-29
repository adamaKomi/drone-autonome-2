sim_vehicle.py -v ArduCopter --console --map --out=127.0.0.1:14550



# Test du drone avec ArduPilot SITL et MAVROS

## 1. Lancer le simulateur ArduPilot SITL
```bash
sim_vehicle.py -v ArduCopter --console --map --out=127.0.0.1:14550
```
Ce simulateur démarre un drone virtuel et diffuse les données MAVLink sur le port 14550.

## 2. Lancer MAVROS (ROS2)
```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
```
MAVROS fait le pont entre ROS2 et le simulateur ArduPilot.

## 3. Vérifier l'état du drone
```bash
ros2 topic echo /mavros/state
```
Ce topic affiche l'état du drone (armé, mode, connexion, etc.).

---
**Astuce :** Utilisez aussi :
```bash
ros2 topic echo /mavros/setpoint_position/local
```
pour vérifier la réception des commandes de position.

