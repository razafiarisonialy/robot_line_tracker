# Robot Suiveur de Ligne — ROS2 + GoPiGo3

Projet ROS2 de suivi de ligne autonome pour robot GoPiGo3.  
Architecture modulaire en 4 nœuds communicants via topics.

## Structure du projet

```
ros2_ws/
└── src/
    └── robot_line_tracker/
        ├── line_tracker_interfaces/   # Messages custom (CMake)
        │   └── msg/
        │       └── LineDetection.msg
        ├── camera/                    # Acquisition & traitement image + FSM
        ├── controller/                # Contrôleur P + logique de récupération
        ├── moteur/                    # Pilotage des moteurs GoPiGo3
        └── line_tracker/              # Package de lancement
```

---

## Architecture logicielle

```
[v4l2_camera_node]
        │ /image_raw (sensor_msgs/Image)
        ▼
[camera_node]  ←── FSM : FOLLOWING / SEARCHING / STOP_LOST
        │ /line/detection (line_tracker_interfaces/LineDetection)
        │ /line/debug     (sensor_msgs/Image)   si debug=True
        │ /line/binary    (sensor_msgs/Image)   si debug=True
        ▼
[controller_node]
        │ /cmd_vel (geometry_msgs/Twist)
        ▼
[motor_node]
        │ → GoPiGo3 (moteurs gauche / droite)
Topics de debug :                                            
   /line/debug  ──► rqt_image_view (image annotée)              
   /line/binary ──► rqt_image_view (image binarisée)       


Topics principaux :
  /image_raw        sensor_msgs/Image        Flux caméra brut
  /line/detection   LineDetection (custom)   État + erreur + direction
  /cmd_vel          geometry_msgs/Twist      Commande vitesse robot

États FSM :
  FOLLOWING (0)  ──► Ligne détectée, suivi actif
  SEARCHING (1)  ──► Ligne perdue < stop_timeout, rotation de recherche
  STOP_LOST (2)  ──► Ligne perdue > stop_timeout, arrêt complet
```

---

## Description des packages

### `line_tracker_interfaces`

Package d'interfaces ROS2 contenant le message personnalisé `LineDetection.msg`.

| Champ | Type | Description |
|---|---|---|
| `line_detected` | bool | Ligne visible dans la ROI |
| `error` | float32 | Distance en pixels entre centre image et centre ligne |
| `state` | uint8 | État FSM : 0=FOLLOWING, 1=SEARCHING, 2=STOP_LOST |
| `last_direction` | float32 | Dernière erreur signée (mémoire directionnelle) |
| `cx_line` | float32 | Coordonnée X du centroïde de la ligne |
| `cx_image` | float32 | Coordonnée X du centre de l'image |

---

### `camera`

Nœud de traitement vision. Reçoit les images brutes, applique le pipeline OpenCV, publie l'état de détection.

**Paramètres configurables :**

| Paramètre | Défaut | Description |
|---|---|---|
| `roi_ratio` | 0.30 | Fraction basse de l'image analysée |
| `use_otsu` | True | Seuillage adaptatif Otsu |
| `threshold` | 80 | Seuil fixe si Otsu désactivé |
| `min_area` | 300 | Aire minimale contour valide (px²) |
| `blur_size` | 7 | Taille kernel Gaussien (impair) |
| `morph_size` | 5 | Taille kernel morphologique |
| `search_timeout` | 1.5 s | Délai avant passage en STOP_LOST |
| `stop_timeout` | 4.0 s | Délai avant arrêt total |
| `min_contrast` | 15.0 | Contraste minimum pour déclencher la détection |
| `max_white_ratio` | 0.80 | Ratio max pixels blancs (anti-surexposition) |
| `use_multi_roi` | True | ROI multi-niveaux pour virages |
| `roi_levels` | 3 | Nombre de niveaux ROI de repli |

---

### `controller`

Nœud de contrôle. Reçoit l'état de détection et publie les commandes de vitesse.

**Paramètres configurables :**

| Paramètre | Défaut | Description |
|---|---|---|
| `kp` | 0.008 | Gain proportionnel |
| `base_speed` | 0.08 m/s | Vitesse linéaire de base |
| `max_angular` | 1.0 rad/s | Vitesse angulaire maximale |
| `speed_reduction` | 0.5 | Coefficient de freinage en virage |
| `min_speed` | 0.04 m/s | Vitesse minimale garantie |
| `dead_band` | 5.0 px | Zone morte autour du centre |
| `curve_threshold` | 0.4 | Seuil d'activation boost virage |
| `curve_boost` | 1.6 | Multiplicateur de rotation en virage |
| `search_angular` | 0.5 rad/s | Vitesse angulaire en recherche |
| `search_ramp_time` | 1.0 s | Durée de rampe montante en recherche |
| `watchdog_timeout` | 1.0 s | Arrêt si aucun message reçu |

---

### `moteur`

Nœud d'actionnement. Traduit les commandes Twist en vitesses DPS moteur GoPiGo3.

**Paramètres configurables :**

| Paramètre | Défaut | Description |
|---|---|---|
| `base_speed` | 200 DPS | Vitesse de base moteur |
| `max_speed` | 400 DPS | Vitesse maximale moteur |
| `steer_gain` | 50.0 | Gain de braquage (0 = cinématique pure) |


---

## Dépendances

### Installation Package

- ROS2 Jazzy : https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Raspberry Pi camera : https://www.youtube.com/watch?v=va7o7wzhEE4
- cv_bridge : https://index.ros.org/p/cv_bridge/#jazzy
- OpenCV : https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html

### Documentation

- v4l2_camera (ROS2 Jazzy) : https://docs.ros.org/en/jazzy/p/v4l2_camera/
- GoPiGo3 : https://gopigo3.readthedocs.io/en/master/api-basic/easygopigo3.html


---

## Installation Projet

### Créer l'espace de travail

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/razafiarisonialy/robot_line_tracker.git
```


### Compiler le workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Sourcer l'environnement

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Pour automatiser, ajouter à `~/.bashrc` :
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```


## Lancement du projet

### Lancement complet (tous les nœuds)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch line_tracker line_tracker.launch.py
```

### Lancement avec paramètres personnalisés

```bash
ros2 launch line_tracker line_tracker.launch.py \
  kp:=0.010 \
  base_speed:=0.06 \
  roi_ratio:=0.35 \
  camera_device:=/dev/video0
```

### Lancement nœud par nœud (debug)

```bash
# Terminal 1 — caméra
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[640,480]

# Terminal 2 — traitement vision
ros2 run camera camera_node --ros-args \
  -p roi_ratio:=0.30 \
  -p use_otsu:=True \
  -p debug:=True

# Terminal 3 — contrôleur
ros2 run controller controller_node --ros-args \
  -p kp:=0.008 \
  -p base_speed:=0.08

# Terminal 4 — moteurs
ros2 run moteur motor_node --ros-args \
  -p base_speed:=200 \
  -p steer_gain:=50.0
```

---

## Exemples d'exécution

### Visualiser le traitement image

```bash
# Ouvrir rqt_image_view
ros2 run rqt_image_view rqt_image_view

# Sélectionner :
# /line/debug  → image couleur annotée
# /line/binary → image binarisée
```

### Inspecter les topics en temps réel

```bash
# Voir les détections de ligne
ros2 topic echo /line/detection

# Voir les commandes moteur
ros2 topic echo /cmd_vel

# Fréquence de publication
ros2 topic hz /line/detection
ros2 topic hz /image_raw
```

### Modifier un paramètre à chaud

```bash
ros2 param set /controller_node kp 0.012
ros2 param set /camera_node roi_ratio 0.40
```

### Graphe des nœuds

```bash
ros2 run rqt_graph rqt_graph
```

---

## Debug & Troubleshooting

### Le robot ne détecte pas la ligne

```bash
# 1. Vérifier que la caméra publie
ros2 topic hz /image_raw

# 2. Visualiser le binaire
ros2 run rqt_image_view rqt_image_view  # → /line/binary

# 3. Ajuster le seuil si image trop sombre
ros2 param set /camera_node threshold 60

# 4. Augmenter la ROI si la ligne n'est pas visible
ros2 param set /camera_node roi_ratio 0.40
```

### Oscillations du robot

```bash
# Réduire le gain proportionnel
ros2 param set /controller_node kp 0.005

# Augmenter la zone morte
ros2 param set /controller_node dead_band 10.0

# Réduire la vitesse de base
ros2 param set /controller_node base_speed 0.06
```

### Robot perd la ligne dans les virages

```bash
# Augmenter les niveaux ROI
ros2 param set /camera_node roi_levels 4

# Augmenter le boost virage
ros2 param set /controller_node curve_boost 1.8

# Baisser le seuil d'activation du boost
ros2 param set /controller_node curve_threshold 0.3
```

