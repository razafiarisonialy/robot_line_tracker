# robot_line_tracker: Robot Suiveur de Ligne — ROS2 / GoPiGo3

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

## Architecture des topics

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
```

---

## Message personnalisé : LineDetection

Le topic `/line/detection` remplace l'ancien `/line/error` (Float32).  
Il transporte un contexte complet pour permettre la logique de récupération :

```
bool    line_detected     # True uniquement si une ligne valide est détectée
float32 error             # Erreur pixel — valide SEULEMENT si line_detected == True
uint8   state             # État FSM : 0=FOLLOWING  1=SEARCHING  2=STOP_LOST
float32 last_direction    # Signe de la dernière erreur connue (direction de récupération)
float32 cx_line           # Centroïde X détecté (debug)
float32 cx_image          # Centre image X (debug)
```

> **Important** : quand `line_detected == False`, `error` vaut `NaN` et non `0.0`.  
> Un `0.0` signifie toujours "ligne parfaitement centrée", jamais "ligne absente".

---

## Machine d'états (FSM)

| État | Condition | Comportement robot |
|------|-----------|--------------------|
| `FOLLOWING` (0) | Ligne détectée | Correcteur P normal — vitesse + cap |
| `SEARCHING` (1) | Ligne absente < `stop_timeout` | Rotation sur place vers la dernière direction connue |
| `STOP_LOST` (2) | Ligne absente > `stop_timeout` | Arrêt complet |

Transitions :
- Ligne retrouvée → retour immédiat à `FOLLOWING`
- Ligne perdue → `SEARCHING` après `search_timeout` secondes
- Toujours perdue → `STOP_LOST` après `stop_timeout` secondes

---

## Dépendances

### Installation

- ROS2 Jazzy : https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Raspberry Pi camera : https://www.youtube.com/watch?v=va7o7wzhEE4
- cv_bridge : https://index.ros.org/p/cv_bridge/#jazzy
- OpenCV : https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html

### Documentation

- v4l2_camera (ROS2 Jazzy) : https://docs.ros.org/en/jazzy/p/v4l2_camera/
- GoPiGo3 : https://gopigo3.readthedocs.io/en/master/api-basic/easygopigo3.html

---

## Build

```bash
mkdir -p ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/razafiarisonialy/robot_line_tracker.git

cd ~/ros2_ws

colcon build --packages-select line_tracker_interfaces --symlink-install && \
source install/setup.bash && \
colcon build --symlink-install
```

> **`--symlink-install`** permet d'éditer les fichiers Python sans rebuild.  
> **L'ordre de build est strict** : `line_tracker_interfaces` doit être compilé avant les packages Python qui en dépendent.

---

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

---

## Vérifier le message custom

```bash
ros2 interface show line_tracker_interfaces/msg/LineDetection
```

Résultat attendu :
```
bool line_detected
float32 error
uint8 state
float32 last_direction
float32 cx_line
float32 cx_image
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
  motor_base_speed:=200 \
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

### `camera_node` — Traitement image + FSM (`package: camera`)

| Paramètre | Type | Défaut | Description |
|-----------|------|--------|-------------|
| `roi_ratio` | float | `0.5` | Fraction de l'image analysée depuis le bas (0.0–1.0) |
| `use_otsu` | bool | `True` | Seuillage adaptatif Otsu |
| `threshold` | int | `80` | Seuil fixe si `use_otsu=False` |
| `min_area` | int | `300` | Surface minimale du contour (px²) |
| `blur_size` | int | `7` | Taille du filtre gaussien (doit être impair) |
| `morph_size` | int | `5` | Taille du noyau morphologique |
| `debug` | bool | `True` | Publier `/line/debug` et `/line/binary` |
| `search_timeout` | float | `3.0` | Délai (s) avant passage à l'état SEARCHING |
| `stop_timeout` | float | `6.0` | Délai (s) avant passage à l'état STOP_LOST |

Topics publiés :

| Topic | Type | Condition |
|-------|------|-----------|
| `/line/detection` | `line_tracker_interfaces/LineDetection` | Toujours |
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

### `controller_node` — Contrôleur P + FSM (`package: controller`)

| Paramètre | Type | Défaut | Description |
|-----------|------|--------|-------------|
| `kp` | float | `0.003` | Gain proportionnel |
| `base_speed` | float | `0.15` | Vitesse linéaire de croisière (m/s) |
| `max_angular` | float | `0.8` | Saturation angulaire (rad/s) |
| `speed_reduction` | float | `0.3` | Réduction de vitesse en virage |
| `min_speed` | float | `0.05` | Vitesse linéaire minimale (m/s) |
| `search_angular` | float | `0.4` | Vitesse de rotation en état SEARCHING (rad/s) |
| `watchdog_period` | float | `0.5` | Période du timer watchdog (s) |
| `watchdog_timeout` | float | `1.0` | Délai sans `/line/detection` avant arrêt watchdog |


Comportement par état :

| État reçu | `linear_x` | `angular_z` |
|-----------|-----------|------------|
| `FOLLOWING` | Correcteur P réduit en virage | `Kp × error` |
| `SEARCHING` | `0.0` (arrêt en translation) | `±search_angular` |
| `STOP_LOST` | `0.0` | `0.0` |

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

> **Note `steer_gain`** : si `steer_gain = 0.0`, le driver calcule le différentiel à partir de la géométrie réelle (empattement = 117 mm, diamètre roue = 66.5 mm).

> **Protection NaN** : si `linear_x` ou `angular_z` est `NaN`, le nœud arrête les moteurs immédiatement.

Exemple via launch file :
```bash
ros2 launch line_tracker line_tracker.launch.py \
  motor_base_speed:=200 \
  motor_max_speed:=350 \
  steer_gain:=50.0
```

Exemple nœud direct :
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
7. **Récupération trop lente** → Réduire `search_timeout` ou augmenter `search_angular`
8. **Robot ne s'arrête jamais** → Réduire `stop_timeout`
9. **Robot tourne du mauvais côté en SEARCHING** → Vérifier le signe de `last_direction` dans les logs
