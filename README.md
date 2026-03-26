# robot_line_tracker: Robot Suiveur de Ligne — ROS2 / GoPiGo3

Projet ROS2 de suivi de ligne autonome pour robot GoPiGo3.  
Architecture modulaire en 4 nœuds communicants via topics.

## Structure du projet

```
ros2_ws/
└── src/
    └── robot_line_tracker/
        ├── camera/          # Acquisition & traitement image
        ├── controller/      # controlleur
        ├── moteur/          # Pilotage des moteurs GoPiGo3
        └── line_tracker/    # Package de lancement
```

---

## Architecture des topics

```
[v4l2_camera_node]
        │ /image_raw (sensor_msgs/Image)
        ▼
[camera_node]
        │ /line/error (std_msgs/Float32)
        ▼
[controller_node]
        │ /cmd_vel (geometry_msgs/Twist)
        ▼
[motor_node]
        │ → GoPiGo3 (moteurs gauche / droite)
```

---

## Dépendances

### Installation

- ROS2 Jazzy installé sur le Raspberry Pi: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- raspberry pi camera: https://www.youtube.com/watch?v=va7o7wzhEE4
- install cv_bridge: https://index.ros.org/p/cv_bridge/#jazzy
- opencv: https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html

### Documentation

- Documentation package v4l2_camera sur ros2 jazzy: https://docs.ros.org/en/jazzy/p/v4l2_camera/
- Documentation gopigo3: https://gopigo3.readthedocs.io/en/master/api-basic/easygopigo3.html

---

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

> **Remarque** : `--symlink-install` permet d'éditer les fichiers Python sans rebuild.

---

## Sourcer l'environnement

```bash
# À exécuter dans chaque nouveau terminal
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Pour automatiser, ajouter à `~/.bashrc` :
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Lancement

### Option 1 — Launch file complet (recommandé)

```bash
ros2 launch line_tracker line_tracker.launch.py
```

Avec paramètres personnalisés (exemple) :
```bash
ros2 launch line_tracker line_tracker.launch.py \
  kp:=0.006 \
  base_speed:=0.12 \
  camera_device:=/dev/video0 \
  debug:=True
```

### Option 2 — Nœuds individuels (debug)

**Terminal 1 — Caméra :**
```bash
ros2 run v4l2_camera v4l2_camera_node
```

**Terminal 2 — Traitement image :**
```bash
ros2 run camera camera_node
```

**Terminal 3 — Contrôleur :**
```bash
ros2 run controller controller_node
```

**Terminal 4 — Moteurs :**
```bash
ros2 run moteur motor_node
```

---

## Paramètres par nœud

### `v4l2_camera_node` — Acquisition caméra

| Paramètre | Type | Défaut | Description |
|-----------|------|--------|-------------|
| `camera_device` | string | `/dev/video0` | Périphérique caméra USB |
| `pixel_format` | string | `YUYV` | Format pixel V4L2 : `YUYV`, `UYVY`, `GREY` |
| `output_encoding` | string | `bgr8` | Encodage de sortie — `bgr8` requis par camera_node |
| `image_size` | int[2] | `[640, 480]` | Résolution de capture en pixels |

Exemple :
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p pixel_format:=YUYV \
  -p output_encoding:=bgr8 \
  -p image_size:=[640,480]
```

---

### `camera_node` — Traitement image (`package: camera`)

| Paramètre | Type | Défaut | Description |
|-----------|------|--------|-------------|
| `roi_ratio` | float | `0.5` | Fraction de l'image analysée depuis le bas (0.0–1.0) |
| `use_otsu` | bool | `True` | Seuillage adaptatif Otsu — recommandé pour éclairage variable |
| `threshold` | int | `80` | Seuil fixe, utilisé uniquement si `use_otsu=False` |
| `min_area` | int | `300` | Surface minimale du contour en px² — filtre les petits artefacts |
| `blur_size` | int | `7` | Taille du filtre gaussien (doit être **impair**) |
| `morph_size` | int | `5` | Taille du noyau morphologique (close + open) |
| `debug` | bool | `True` | Publier les images annotées sur `/line/debug` et `/line/binary` |

Topics publiés :

| Topic | Type | Condition |
|-------|------|-----------|
| `/line/error` | `std_msgs/Float32` | Toujours |
| `/line/debug` | `sensor_msgs/Image` | Si `debug=True` |
| `/line/binary` | `sensor_msgs/Image` | Si `debug=True` |

Exemple :
```bash
ros2 run camera camera_node --ros-args \
  -p roi_ratio:=0.4 \
  -p use_otsu:=true \
  -p min_area:=200 \
  -p blur_size:=7 \
  -p morph_size:=5 \
  -p debug:=true
```

---

### `controller_node` — controlleur (`package: controller`)

| Paramètre | Type | Défaut | Description |
|-----------|------|--------|-------------|
| `kp` | float | `0.005` | Gain proportionnel — augmenter si réponse lente, réduire si oscillations |
| `base_speed` | float | `0.15` | Vitesse linéaire de croisière en m/s |
| `max_angular` | float | `0.8` | Saturation de la vitesse angulaire en rad/s |
| `speed_reduction` | float | `0.5` | Réduction de vitesse en virage (`0.0` = aucune, `1.0` = arrêt complet) |
| `min_speed` | float | `0.05` | Vitesse linéaire minimale en m/s — évite l'arrêt complet |
| `watchdog_period` | float | `0.5` | Période du timer watchdog en secondes |
| `watchdog_timeout` | float | `1.0` | Délai sans message `/line/error` avant alerte watchdog |

Exemple :
```bash
ros2 run controller controller_node --ros-args \
  -p kp:=0.006 \
  -p base_speed:=0.12 \
  -p max_angular:=0.8 \
  -p speed_reduction:=0.6 \
  -p min_speed:=0.05
```

---

### `motor_node` — Pilotage moteurs (`package: moteur`)

| Paramètre | Type | Défaut | Description |
|-----------|------|--------|-------------|
| `base_speed` | int | `200` | Vitesse de base des moteurs en DPS (degrés/seconde) |
| `max_speed` | int | `400` | Vitesse maximale autorisée en DPS |
| `steer_gain` | float | `50.0` | Gain de braquage : `diff = angular_z × steer_gain`. Mettre à `0.0` pour la cinématique différentielle réelle |
| `watchdog_period` | float | `0.5` | Période du timer watchdog en secondes |
| `watchdog_timeout` | float | `1.0` | Délai sans `/cmd_vel` avant arrêt d'urgence des moteurs |

> **Note `steer_gain`** : si `steer_gain = 0.0`, le driver calcule le différentiel de vitesse à partir de la géométrie réelle du robot (empattement = 117 mm, diamètre roue = 66.5 mm).

Exemple :
```bash
ros2 run moteur motor_node --ros-args \
  -p base_speed:=200 \
  -p max_speed:=400 \
  -p steer_gain:=50.0
```

---

## Calibration rapide

1. **Ligne non détectée** → Réduire `min_area` ou ajuster l'éclairage
2. **Oscillations** → Réduire `kp`
3. **Réponse trop lente** → Augmenter `kp`
4. **Perd la ligne dans les virages** → Réduire `base_speed`, augmenter `roi_ratio`
5. **Binarisation bruitée** → Augmenter `blur_size` (valeur impaire) ou `morph_size`
6. **Robot trop lent en ligne droite** → Augmenter `base_speed` ou réduire `speed_reduction`
