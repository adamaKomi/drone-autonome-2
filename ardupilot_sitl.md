# Simulation de la batterie et du GPS dans ArduPilot SITL

## Batterie

**Augmenter la tension**
```bash
param set SIM_BATT_VOLTAGE 12.6
```

**Augmenter le courant**
```bash
param set BATT_CAPACITY 10000 # Changer la valeur selon vos besoins
```

---

## Paramètres GPS pour la simulation SITL

Voici un tableau récapitulatif des paramètres GPS dans SITL (ArduPilot) pour simuler différents scénarios :

| Paramètre         | Description                                 | Valeur typique | Effet                                      |
|-------------------|---------------------------------------------|---------------|---------------------------------------------|
| SIM_GPS1_ENABLE   | Active/désactive le GPS n°1                 | 1 (activé)    | 0 = perte totale du GPS 1                   |
| SIM_GPS2_ENABLE   | Active/désactive le GPS n°2 (si configuré)  | 1 ou 0        | Tester le failover GPS                      |
| SIM_GPS1_NUMSATS  | Nombre de satellites GPS 1                  | 10            | 0 = perte de fix                            |
| SIM_GPS2_NUMSATS  | Nombre de satellites GPS 2                  | 10            | Simuler GPS secondaire plus/moins fiable     |
| SIM_GPS1_HDOP     | HDOP GPS 1 (précision horizontale)          | 1.0           | 10+ = mauvaise précision                    |
| SIM_GPS2_HDOP     | HDOP GPS 2                                  | 1.0           | Simuler GPS secondaire imprécis              |
| SIM_GPS1_DELAY    | Délai de latence GPS 1 (secondes)           | 0.2           | Augmenter = lag GPS                         |
| SIM_GPS2_DELAY    | Délai de latence GPS 2                      | 0.2           | Tester désynchronisation                    |
| SIM_GPS1_TYPE     | Type de GPS simulé (0=aucun, 1=UBlox, etc.) | 1             | Forcer un type spécifique                   |
| SIM_GPS2_TYPE     | Type de GPS simulé pour GPS 2               | 1             | Tester une config différente                |

---

## Exemples pratiques

🔹 **Désactiver totalement le GPS 1**
```bash
param set SIM_GPS1_ENABLE 0
```

🔹 **Simuler une perte de satellites**
```bash
param set SIM_GPS1_NUMSATS 0
```

🔹 **Mauvais signal GPS (imprécision)**
```bash
param set SIM_GPS1_HDOP 15
```

🔹 **Tester un délai GPS (lag)**
```bash
param set SIM_GPS1_DELAY 2
```

---

## Simulation du GPS et armement dans SITL

Pour que le drone puisse s'armer dans ArduPilot SITL, il doit avoir un "GPS fix" fiable :
- Le nombre de satellites doit être suffisant (généralement ≥ 6).
- La précision (HDOP) doit être correcte.

**Exemple pratique**

Si le message `AP: PreArm: AHRS: waiting for home` apparaît, cela signifie que le drone attend une position "home" valide (fix GPS).

Pour résoudre ce problème :
- Vérifiez le nombre de satellites :
  ```bash
  param show SIM_GPS1_NUMSATS
  ```
- Augmentez le nombre de satellites simulés :
  ```bash
  param set SIM_GPS1_NUMSATS 12
  ```

Après cette commande, le message "waiting for home" disparaît et le drone peut être armé normalement.

Pour simuler une perte de GPS, baissez ce nombre (ex. : `param set SIM_GPS1_NUMSATS 0`).

---
