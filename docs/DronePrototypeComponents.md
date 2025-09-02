# Projet : Drone autonome pour la détection et la pollinisation de fleurs
# Résumé des échanges pour la conception d'un prototype
# Localisation : Maroc
# Objectif : Construire un prototype économique pour prouver la faisabilité du projet, sans priorité sur les hautes performances (autonomie, portée, etc.).

## Critères pour le prototype
- Vol autonome avec planification de trajectoires (navigation par waypoints).
- Détection visuelle des fleurs via caméra (ex. : traitement d'image simple).
- Stabilisation précise pour l'approche des fleurs.
- Mécanisme de pollinisation simulé (ex. : servo léger).
- Collecte de données basique (positions GPS, logs).
- Budget abordable, composants accessibles au Maroc (via AliExpress).
- Cadre compact (3 pouces) pour tests en intérieur/extérieur.

## Contrôleur de vol recommandé
- **Choix final** : SpeedyBee F405 V4
  - Raison : Économique, compatible avec ArduPilot (pour missions autonomes), ESC intégré (55A 4-in-1), ports UART pour GPS et caméra, configuration facile via Bluetooth.
  - Caractéristiques : Processeur STM32F405 (168 MHz), IMU intégré, supporte GPS externe, journalisation sur carte SD.
  - Prix : ~39,99 $ (AliExpress).
  - Lien : https://www.aliexpress.com/item/1005004737031055.html

## Composants sélectionnés (compatibles)
1. **Structure (Frame)** :
   - Modèle : R3micro 3-Inch Professional FPV Racing Drone Frame
   - Caractéristiques : Léger, fibre de carbone, montage 30x30mm, adapté aux moteurs 1104 et hélices 3 pouces.
   - Prix : ~17,99 $ (AliExpress).
   - Lien : https://www.aliexpress.com/w/wholesale-3-inch-fpv-drone-frame.html
   - Compatibilité : Supporte le contrôleur de vol, moteurs, batterie, et récepteur.

2. **Moteurs** :
   - Modèle : T-Motor M1104 1104 7500KV FPV Drone Motor (set de 4)
   - Caractéristiques : Légers (5,6 g chacun), puissants, adaptés aux cadres 3 pouces, compatibles avec batterie 2S/3S.
   - Prix : ~19,99 $ (AliExpress).
   - Lien : https://www.aliexpress.com/w/wholesale-1104-motor.html
   - Compatibilité : Connectés à l'ESC intégré du SpeedyBee F405 V4.

3. **Hélices** :
   - Modèle : HQProp 3x3.6x3 75mm Cinewhoop Propeller Set (set de 4)
   - Caractéristiques : 3 pouces, légères, optimisées pour stabilité et maniabilité.
   - Prix : ~3,39 $ (AliExpress).
   - Lien : https://www.aliexpress.com/w/wholesale-3-inch-drone-propellers.html
   - Compatibilité : Adaptées aux moteurs M1104.

4. **Récepteur radio** :
   - Modèle : FrSky XSR Receiver
   - Caractéristiques : Compact, protocole SBUS, portée 1-2 km, pour contrôle manuel ou supervision autonome.
   - Prix : ~19,99 $ (AliExpress).
   - Lien : https://www.aliexpress.com/w/wholesale-frsky-receiver.html
   - Compatibilité : Connecté via UART au contrôleur de vol.

5. **Batterie** :
   - Modèle : Tattu R-Line 1000mAh 2S 7.6V 95C HV Lipo Battery Pack with XT30 plug
   - Caractéristiques : Légère (44,4 g), autonomie ~5-10 min, compatible avec moteurs et contrôleur.
   - Prix : ~12,99 $ (AliExpress).
   - Lien : https://www.aliexpress.com/w/wholesale-lipo-battery-2s-1000mah.html
   - Compatibilité : Connecteur XT30, tension adaptée (2S).

## Coût total estimé
- Contrôleur de vol : 39,99 $
- Structure : 17,99 $
- Moteurs : 19,99 $
- Hélices : 3,39 $
- Récepteur radio : 19,99 $
- Batterie : 12,99 $
- **Total** : ~114,34 $ (hors frais de livraison et douanes)

## Où acheter au Maroc
- **AliExpress** : Plateforme principale, livraison au Maroc (7-15 jours selon l'option). Vérifier les vendeurs (avis, réputation) et les frais de port.
  - Exemple : DHL ou EMS pour une livraison rapide.
- **Banggood** : Alternative en ligne avec livraison au Maroc, composants similaires.
- **eBay** : Moins recommandé (frais de port élevés), mais possible.
- **Local (Casablanca, Rabat)** : Magasins de modélisme (ex. : Droneway pour DJI) ou d'électronique peuvent commander des pièces, mais disponibilité limitée. Contacter pour confirmation.
- **Conseil** : Privilégier AliExpress pour le prix et la disponibilité. Vérifier les taxes douanières marocaines (Direction Générale des Douanes et Impôts Indirects).

## Compatibilité des composants
- **Structure** : Supporte le montage 30x30mm du SpeedyBee F405 V4, moteurs M1104, et espace pour batterie/récepteur.
- **Contrôleur de vol** : ESC intégré (55A) gère les moteurs M1104, ports UART pour récepteur et capteurs (GPS, caméra).
- **Moteurs/Hélices** : M1104 compatibles avec hélices 3 pouces, alimentés par batterie 2S via l'ESC.
- **Récepteur** : Connecté via SBUS/UART, compatible avec émetteurs FrSky.
- **Batterie** : Tension 2S adaptée au contrôleur et aux moteurs, connecteur XT30 standard.

## Conseils pour le prototype
- **Firmware** : Utiliser ArduPilot sur le SpeedyBee F405 V4 pour les missions autonomes (waypoints, RTL). Configurer via Mission Planner.
- **Capteurs supplémentaires** (non inclus dans le coût pour minimiser les dépenses) :
  - GPS : Matek M8Q-5883 (~25 $) pour navigation et collecte de positions.
  - Caméra : Runcam Phoenix 2 (~20 $) pour détection visuelle basique.
  - Ajouter si nécessaire pour les tests de détection.
- **Mécanisme de pollinisation** : Utiliser un micro-servo (ex. : Emax ES08MA, ~5 $) pour un bras simulé, connecté via PWM.
- **Tests** : Commencer en intérieur avec protections d'hélices. Configurer une géobarrière et un arrêt d'urgence via ArduPilot.
- **Configuration** : Télécharger Mission Planner pour paramétrer le contrôleur (ex. : vitesse de navigation 500 cm/s, rayon waypoint 100 cm).
- **Sécurité** : Vérifier les vibrations (via Mission Planner) pour éviter les interférences avec l'IMU.

## Remarques
- Les prix peuvent varier (promotions, frais de port, douanes). Vérifier sur AliExpress avant commande.
- Prévoir 1-2 semaines pour la livraison au Maroc.
- Pour une intégration AI (ex. : YOLO), ajouter un Raspberry Pi 4 (~35 $) plus tard, une fois la faisabilité prouvée.
- Contacter des magasins locaux (ex. : Droneway à Casablanca) pour vérifier la disponibilité ou la commande de pièces.