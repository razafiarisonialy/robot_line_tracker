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

---

### `camera`

Nœud de traitement vision. Reçoit les images brutes, applique le pipeline OpenCV, publie l'état de détection.

**Paramètres configurables :**

| Paramètre | Défaut | Description |
|---|---|---|
| `roi_ratio` | 0.30 | Fraction basse de l'image analysée [0.15–0.55] |
| `use_otsu` | True | Seuillage adaptatif Otsu |
| `threshold` | 80 | Seuil fixe si Otsu désactivé [0–255] |
| `min_area` | 300 | Aire minimale contour valide (px²) [100–800] |
| `blur_size` | 7 | Taille kernel Gaussien (impair) [3,5,7,9] |
| `morph_size` | 5 | Taille kernel morphologique [3–9] |
| `debug` | True | Publie /line/debug et /line/binary |
| `search_timeout` | 1.5 s | Délai avant passage en SEARCHING après perte de ligne [0.5–3.0] |
| `stop_timeout` | 4.0 s | Délai avant arrêt total STOP_LOST [2.0–8.0] |

---

### `controller`

Nœud de contrôle. Reçoit l'état de détection et publie les commandes de vitesse.

**Paramètres configurables :**

| Paramètre | Défaut | Description |
|---|---|---|
| `kp` | 0.0071 | Gain proportionnel (erreur px → rad/s) [0.003–0.020] |
| `base_speed` | 0.07 m/s | Vitesse linéaire de base [0.03–0.15] |
| `max_angular` | 3.0 rad/s | Saturation angulaire maximale [0.5–5.0] |
| `speed_reduction` | 0.6 | Coefficient de réduction vitesse en virage [0.3–0.8] |
| `min_speed` | 0.0 m/s | Vitesse linéaire minimale garantie [0.0–0.06] |
| `search_angular` | 0.55 rad/s | Vitesse angulaire en mode SEARCHING [0.2–1.0] |
| `sharp_turn_threshold` | 80.0 px | Seuil erreur pour détecter un virage serré [40–150] |
| `sharp_turn_boost` | 2.5 | Multiplicateur du Kp en virage serré [1.5–4.0] |
| `watchdog_period` | 0.5 s | Période de vérification watchdog [0.1–1.0] |
| `watchdog_timeout` | 1.0 s | Arrêt si aucun message reçu [0.5–3.0] |

---

### `moteur`

Nœud d'actionnement. Traduit les commandes Twist en vitesses DPS moteur GoPiGo3.

**Constantes physiques :** diamètre roue = 66.5 mm, empattement = 117.0 mm.

**Paramètres configurables :**

| Paramètre | Défaut | Description |
|---|---|---|
| `max_speed` | 400 DPS | Vitesse maximale moteur [200–700] |
| `steer_gain` | 68.22 | Gain empirique de braquage (DPS/rad/s). 0 = cinématique pure [0–150] |
| `watchdog_period` | 0.5 s | Période de vérification watchdog moteur [0.1–1.0] |
| `watchdog_timeout` | 1.0 s | Timeout watchdog moteur [0.5–3.0] |

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
ros2 launch line_tracker line_tracker.launch.py
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
  -p kp:=0.0071 \
  -p base_speed:=0.07

# Terminal 4 — moteurs
ros2 run moteur motor_node --ros-args \
  -p max_speed:=400 \
  -p steer_gain:=68.22
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
ros2 param set /motor_node steer_gain 55.0
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
ros2 param set /controller_node kp 0.004

# Réduire la vitesse de base
ros2 param set /controller_node base_speed 0.05
```

### Robot perd la ligne dans les virages

```bash

# Augmenter le boost virage (augmente vitesse angulaire)
ros2 param set /controller_node steer_gain 80.0

# Augmenter la ROI
ros2 param set /camera_node roi_ratio 0.50
```

### Robot trop lent

```bash
# Augmenter la vitesse de base
ros2 param set /controller_node base_speed 0.10

# Augmenter la vitesse max moteur
ros2 param set /motor_node max_speed 500
```

### Robot s'arrête trop vite après perte de ligne

```bash
# Augmenter le délai avant arrêt
ros2 param set /camera_node stop_timeout 6.0
```
