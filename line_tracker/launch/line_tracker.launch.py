from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # v4l2_camera_node — Acquisition caméra
    arg_camera_device   = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Périphérique V4L2 de la caméra'
    )
    arg_pixel_format    = DeclareLaunchArgument(
        'pixel_format',
        default_value='YUYV',
        description='Format pixel natif de la caméra (YUYV, MJPG)'
    )
    arg_output_encoding = DeclareLaunchArgument(
        'output_encoding',
        default_value='bgr8',
        description='Encodage de sortie ROS2 (bgr8 requis par OpenCV)'
    )


    # camera_node — Traitement vision

    # ── ROI 
    arg_roi_ratio = DeclareLaunchArgument(
        'roi_ratio',
        default_value='0.35',
        # ce qui améliore l'anticipation des virages à 90°.
        # Trop grand (>0.55) : capte des objets parasites hors-sol.
        description='[0.15–0.55] Fraction basse de l image analysée'
    )

    # ── Binarisation ────────────────────────────────────────────
    arg_use_otsu = DeclareLaunchArgument(
        'use_otsu',
        default_value='True',
        description='True=seuillage Otsu adaptatif, False=seuil fixe (threshold)'
    )
    arg_threshold = DeclareLaunchArgument(
        'threshold',
        default_value='80',
        description='[0–255] Seuil fixe si use_otsu=False (ligne noire sur fond clair → 80–120)'
    )

    # ── Filtrage ─────────────────────────────────────────────────
    arg_blur_size = DeclareLaunchArgument(
        'blur_size',
        default_value='7',
        description='[3,5,7,9] Taille noyau gaussien (impair obligatoire). 7 = bon compromis bruit/détail'
    )
    arg_morph_size = DeclareLaunchArgument(
        'morph_size',
        default_value='5',
        description='[3–9] Taille noyau morphologique. >7 risque d effacer les lignes fines'
    )

    # ── Validation contour ───────────────────────────────────────
    arg_min_area = DeclareLaunchArgument(
        'min_area',
        default_value='250',
        # dans les virages ou quand la ROI est partiellement hors-piste.
        # Trop bas (<100) : bruit et reflets deviennent des faux positifs.
        description='[100–800] Aire minimale (px²) d un contour valide'
    )

    # ── Robustesse éclairage ─────────────────────────────────────
    arg_min_contrast = DeclareLaunchArgument(
        'min_contrast',
        default_value='12.0',
        # tolère un éclairage intérieur faible
        # Trop bas (<5) : déclenche sur une image uniforme bruitée.
        description='[5.0–30.0] Ecart-type min des niveaux de gris (anti-image-vide)'
    )
    arg_max_white_ratio = DeclareLaunchArgument(
        'max_white_ratio',
        default_value='0.80',
        # invalide la frame si >80% pixels blancs
        # (surexposition ou sol très clair). Augmenter si le sol est naturellement
        # très clair (carrelage blanc).
        description='[0.50–0.95] Ratio max pixels blancs post-binarisation (anti-surexposition)'
    )

    # ── Debug ────────────────────────────────────────────────────
    arg_debug = DeclareLaunchArgument(
        'debug',
        default_value='True',
        description='True=publie /line/debug et /line/binary (désactiver en production)'
    )

    # ── FSM timeouts ─────────────────────────────────────────────
    arg_search_timeout = DeclareLaunchArgument(
        'search_timeout',
        default_value='1.5',
        description='[0.5–3.0] Délai (s) avant passage en SEARCHING après perte de ligne'
    )
    arg_stop_timeout = DeclareLaunchArgument(
        'stop_timeout',
        default_value='4.0',
        description='[2.0–8.0] Délai (s) avant STOP_LOST (arrêt complet)'
    )

    # ── ROI multi-niveaux ────────────────────────────────────────
    arg_use_multi_roi = DeclareLaunchArgument(
        'use_multi_roi',
        default_value='True',
        description='True=fallback vers ROI plus haute si ligne non trouvée (aide les virages)'
    )
    arg_roi_levels = DeclareLaunchArgument(
        'roi_levels',
        default_value='3',
        description='[1–5] Nombre de niveaux ROI de repli (1=désactivé, >4 augmente la latence)'
    )

    # controller_node — Loi de commande

    # ── Gain proportionnel ───────────────────────────────────────
    arg_kp = DeclareLaunchArgument(
        'kp',
        default_value='0.008',
        # 0.008 est calibré pour 640px de large et max_angular=1.0.
        # Formule de départ : kp ≈ max_angular / (image_width/2) = 1.0 / 320 ≈ 0.003 (min théorique)
        # 0.008 donne une réponse plus vive sans osciller à 0.08 m/s.
        # Augmenter si le robot est trop mou ; réduire si oscillations.
        description='[0.003–0.020] Gain proportionnel (erreur px → rad/s)'
    )

    # ── Vitesses ─────────────────────────────────────────────────
    arg_base_speed = DeclareLaunchArgument(
        'base_speed',
        default_value='0.08',
        description='[0.03–0.15] Vitesse linéaire de base (m/s). Réduire pour virages serrés'
    )
    arg_max_angular = DeclareLaunchArgument(
        'max_angular',
        default_value='1.0',
        # Augmenté permet des virages U plus serrés.
        # Le GoPiGo3 peut atteindre ~2.0 rad/s mais l inertie limite l utilité.
        description='[0.5–2.0] Saturation angulaire maximale (rad/s)'
    )
    arg_min_speed = DeclareLaunchArgument(
        'min_speed',
        default_value='0.03',
        # Réduit 0.04 → 0.03 : en virage très serré le robot peut ralentir davantage,
        # réduisant le risque de sortir de piste. min_speed=0 est risqué (robot bloqué).
        description='[0.02–0.06] Vitesse linéaire minimale garantie (m/s)'
    )

    # ── Loi de réduction vitesse ─────────────────────────────────
    arg_speed_reduction = DeclareLaunchArgument(
        'speed_reduction',
        default_value='0.6',
        # Augmenté 0.5 → 0.6 : ralentit plus agressivement en virage.
        # Avec error_norm=1 : linear_x = base × (1 - 0.6×1²) = base × 0.4
        # = 0.08 × 0.4 = 0.032 m/s (proche de min_speed → bon)
        # Formule : linear_x_min_effective = base_speed × (1 - speed_reduction)
        description='[0.3–0.8] Coefficient de freinage quadratique en virage'
    )

    # ── Zone morte ───────────────────────────────────────────────
    arg_dead_band = DeclareLaunchArgument(
        'dead_band',
        default_value='6.0',
        # supprime les micro-corrections dues
        # au bruit de détection (~2–4px) et aux vibrations mécaniques.
        # Sur 640px : 6px = ~0.9% de la largeur = imperceptible visuellement.
        description='[2.0–15.0] Zone morte angulaire (px). Supprime les micro-oscillations'
    )

    # ── Boost virage ─────────────────────────────────────────────
    arg_curve_threshold = DeclareLaunchArgument(
        'curve_threshold',
        default_value='0.35',
        # active le boost plus tôt dans le virage.
        # error_norm > 0.35 signifie erreur > 35% du max → déjà un virage notable.
        description='[0.2–0.6] Seuil error_norm pour activer le boost angulaire'
    )
    arg_curve_boost = DeclareLaunchArgument(
        'curve_boost',
        default_value='1.7',
        # correction plus franche en virage U.
        # curve_boost × max_angular doit rester ≤ 2.0 rad/s
        description='[1.0–2.5] Multiplicateur angulaire en virage (après clip ±max_angular)'
    )

    # ── Recherche de ligne ───────────────────────────────────────
    arg_search_angular = DeclareLaunchArgument(
        'search_angular',
        default_value='0.55',
        # Légèrement augmenté 0.5 → 0.55 : retrouve la ligne plus vite
        # après un virage U. Trop élevé (>0.8) : le robot dépasse et perd de nouveau.
        description='[0.2–1.0] Vitesse angulaire maximale en mode SEARCHING (rad/s)'
    )
    arg_search_ramp_time = DeclareLaunchArgument(
        'search_ramp_time',
        default_value='0.8',
        # Réduit 1.0 → 0.8s : atteint la vitesse de recherche max plus vite.
        # Important pour les virages U rapides où chaque dixième de seconde compte.
        description='[0.2–2.0] Durée de montée en vitesse en mode SEARCHING (s)'
    )

    # ── Watchdog controller ──────────────────────────────────────
    arg_ctrl_watchdog_period = DeclareLaunchArgument(
        'ctrl_watchdog_period',
        default_value='0.5',
        description='[0.1–1.0] Période de vérification watchdog controller (s)'
    )
    arg_ctrl_watchdog_timeout = DeclareLaunchArgument(
        'ctrl_watchdog_timeout',
        default_value='1.0',
        description='[0.5–3.0] Timeout watchdog controller : arrêt si camera silencieux (s)'
    )

    # motor_node — Actionnement GoPiGo3

    arg_motor_base_speed = DeclareLaunchArgument(
        'motor_base_speed',
        default_value='200',
        description='[50–350] Vitesse de base moteur (DPS). Référence de conversion v_ms→DPS'
    )
    arg_motor_max_speed = DeclareLaunchArgument(
        'motor_max_speed',
        default_value='400',
        # donne plus de marge pour les différentiels de virage.
        # Le firmware GoPiGo3 limite à ~1000 DPS ; 450 est conservateur et sûr.
        description='[200–700] Vitesse maximale moteur (DPS). Protège le firmware GoPiGo3'
    )
    arg_steer_gain = DeclareLaunchArgument(
        'steer_gain',
        default_value='50.0',
        # améliore la réactivité en virage avec les
        description='[0–150] Gain empirique de braquage (DPS/rad/s). 0=cinématique pure'
    )

    # ── Watchdog motor ───────────────────────────────────────────
    arg_motor_watchdog_period = DeclareLaunchArgument(
        'motor_watchdog_period',
        default_value='0.5',
        description='[0.1–1.0] Période de vérification watchdog moteur (s)'
    )
    arg_motor_watchdog_timeout = DeclareLaunchArgument(
        'motor_watchdog_timeout',
        default_value='1.0',
        description='[0.5–3.0] Timeout watchdog moteur : arrêt si controller silencieux (s)'
    )

    # Nœuds
    v4l2_camera_node = Node(
        package    = 'v4l2_camera',
        executable = 'v4l2_camera_node',
        name       = 'v4l2_camera_node',
        output     = 'screen',
        parameters = [{
            'video_device':    LaunchConfiguration('camera_device'),
            'pixel_format':    LaunchConfiguration('pixel_format'),
            'output_encoding': LaunchConfiguration('output_encoding'),
            'image_size':      [640, 480],
        }],
    )

    camera_node = Node(
        package    = 'camera',
        executable = 'camera_node',
        name       = 'camera_node',
        output     = 'screen',
        parameters = [{
            # ROI
            'roi_ratio':       LaunchConfiguration('roi_ratio'),
            'use_multi_roi':   LaunchConfiguration('use_multi_roi'),
            'roi_levels':      LaunchConfiguration('roi_levels'),
            # Binarisation
            'use_otsu':        LaunchConfiguration('use_otsu'),
            'threshold':       LaunchConfiguration('threshold'),
            # Filtrage
            'blur_size':       LaunchConfiguration('blur_size'),
            'morph_size':      LaunchConfiguration('morph_size'),
            # Validation
            'min_area':        LaunchConfiguration('min_area'),
            'min_contrast':    LaunchConfiguration('min_contrast'),    
            'max_white_ratio': LaunchConfiguration('max_white_ratio'), 
            # FSM
            'search_timeout':  LaunchConfiguration('search_timeout'),
            'stop_timeout':    LaunchConfiguration('stop_timeout'),
            # Debug
            'debug':           LaunchConfiguration('debug'),
        }],
    )

    controller_node = Node(
        package    = 'controller',
        executable = 'controller_node',
        name       = 'controller_node',
        output     = 'screen',
        parameters = [{
            # Loi de commande
            'kp':               LaunchConfiguration('kp'),
            'base_speed':       LaunchConfiguration('base_speed'),
            'max_angular':      LaunchConfiguration('max_angular'),
            'min_speed':        LaunchConfiguration('min_speed'),
            'speed_reduction':  LaunchConfiguration('speed_reduction'),
            'dead_band':        LaunchConfiguration('dead_band'),       
            # Boost virage
            'curve_threshold':  LaunchConfiguration('curve_threshold'),
            'curve_boost':      LaunchConfiguration('curve_boost'),
            # Recherche
            'search_angular':   LaunchConfiguration('search_angular'),
            'search_ramp_time': LaunchConfiguration('search_ramp_time'),
            # Watchdog
            'watchdog_period':  LaunchConfiguration('ctrl_watchdog_period'), 
            'watchdog_timeout': LaunchConfiguration('ctrl_watchdog_timeout'),
        }],
    )

    motor_node = Node(
        package    = 'moteur',
        executable = 'motor_node',
        name       = 'motor_node',
        output     = 'screen',
        parameters = [{
            'base_speed':       LaunchConfiguration('motor_base_speed'),
            'max_speed':        LaunchConfiguration('motor_max_speed'),
            'steer_gain':       LaunchConfiguration('steer_gain'),
            'watchdog_period':  LaunchConfiguration('motor_watchdog_period'), 
            'watchdog_timeout': LaunchConfiguration('motor_watchdog_timeout'),
        }],
    )

    return LaunchDescription([
        # Caméra
        arg_camera_device, arg_pixel_format, arg_output_encoding,
        # Vision — ROI
        arg_roi_ratio, arg_use_multi_roi, arg_roi_levels,
        # Vision — Binarisation & filtrage
        arg_use_otsu, arg_threshold, arg_blur_size, arg_morph_size,
        # Vision — Validation & robustesse
        arg_min_area, arg_min_contrast, arg_max_white_ratio,
        # Vision — FSM & debug
        arg_search_timeout, arg_stop_timeout, arg_debug,
        # Contrôleur — Loi de commande
        arg_kp, arg_base_speed, arg_max_angular, arg_min_speed,
        arg_speed_reduction, arg_dead_band,
        # Contrôleur — Boost virage
        arg_curve_threshold, arg_curve_boost,
        # Contrôleur — Recherche
        arg_search_angular, arg_search_ramp_time,
        # Contrôleur — Watchdog
        arg_ctrl_watchdog_period, arg_ctrl_watchdog_timeout,
        # Moteur
        arg_motor_base_speed, arg_motor_max_speed, arg_steer_gain,
        # Moteur — Watchdog
        arg_motor_watchdog_period, arg_motor_watchdog_timeout,
        # Nœuds
        v4l2_camera_node,
        camera_node,
        controller_node,
        motor_node,
    ])