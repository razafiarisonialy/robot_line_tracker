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
    arg_roi_ratio = DeclareLaunchArgument(
        'roi_ratio',
        default_value='0.35',
        description='[0.15–0.55] Fraction basse de l image analysée'
    )
    arg_use_otsu = DeclareLaunchArgument(
        'use_otsu',
        default_value='True',
        description='True=seuillage Otsu adaptatif, False=seuil fixe (threshold)'
    )
    arg_threshold = DeclareLaunchArgument(
        'threshold',
        default_value='80',
        description='[0–255] Seuil fixe si use_otsu=False'
    )
    arg_blur_size = DeclareLaunchArgument(
        'blur_size',
        default_value='7',
        description='[3,5,7,9] Taille noyau gaussien (impair obligatoire)'
    )
    arg_morph_size = DeclareLaunchArgument(
        'morph_size',
        default_value='5',
        description='[3–9] Taille noyau morphologique'
    )
    arg_min_area = DeclareLaunchArgument(
        'min_area',
        default_value='250',
        description='[100–800] Aire minimale (px²) d un contour valide'
    )
    arg_debug = DeclareLaunchArgument(
        'debug',
        default_value='True',
        description='True=publie /line/debug et /line/binary'
    )
    arg_search_timeout = DeclareLaunchArgument(
        'search_timeout',
        default_value='1.5',
        description='[0.5–3.0] Délai (s) avant passage en SEARCHING après perte de ligne'
    )
    arg_stop_timeout = DeclareLaunchArgument(
        'stop_timeout',
        default_value='4.0',
        description='[2.0–8.0] Délai (s) avant STOP_LOST (arrêt complet, dernier recours)'
    )


    # controller_node — Loi de commande

    arg_kp = DeclareLaunchArgument(
        'kp',
        default_value='0.008',
        description='[0.003–0.020] Gain proportionnel (erreur px → rad/s)'
    )
    arg_base_speed = DeclareLaunchArgument(
        'base_speed',
        default_value='0.08',
        description='[0.03–0.15] Vitesse linéaire de base (m/s)'
    )
    arg_max_angular = DeclareLaunchArgument(
        'max_angular',
        default_value='1.0',
        description='[0.5–2.0] Saturation angulaire maximale (rad/s)'
    )
    arg_min_speed = DeclareLaunchArgument(
        'min_speed',
        default_value='0.03',
        description='[0.02–0.06] Vitesse linéaire minimale garantie (m/s)'
    )
    arg_speed_reduction = DeclareLaunchArgument(
        'speed_reduction',
        default_value='0.6',
        description='[0.3–0.8] Coefficient de réduction vitesse en virage'
    )
    arg_search_angular = DeclareLaunchArgument(
        'search_angular',
        default_value='0.55',
        description='[0.2–1.0] Vitesse angulaire en mode SEARCHING (rad/s)'
    )
    arg_ctrl_watchdog_period = DeclareLaunchArgument(
        'ctrl_watchdog_period',
        default_value='0.5',
        description='[0.1–1.0] Période de vérification watchdog controller (s)'
    )
    arg_ctrl_watchdog_timeout = DeclareLaunchArgument(
        'ctrl_watchdog_timeout',
        default_value='1.0',
        description='[0.5–3.0] Timeout watchdog controller (s)'
    )



    # motor_node — Actionnement GoPiGo3

    arg_motor_max_speed = DeclareLaunchArgument(
        'motor_max_speed',
        default_value='400',
        description='[200–700] Vitesse maximale moteur (DPS)'
    )
    arg_steer_gain = DeclareLaunchArgument(
        'steer_gain',
        default_value='50.0',
        description='[0–150] Gain empirique de braquage (DPS/rad/s). 0=cinématique pure'
    )
    arg_motor_watchdog_period = DeclareLaunchArgument(
        'motor_watchdog_period',
        default_value='0.5',
        description='[0.1–1.0] Période de vérification watchdog moteur (s)'
    )
    arg_motor_watchdog_timeout = DeclareLaunchArgument(
        'motor_watchdog_timeout',
        default_value='1.0',
        description='[0.5–3.0] Timeout watchdog moteur (s)'
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
            'roi_ratio':  LaunchConfiguration('roi_ratio'),
            'use_otsu':   LaunchConfiguration('use_otsu'),
            'threshold':  LaunchConfiguration('threshold'),
            'blur_size':  LaunchConfiguration('blur_size'),
            'morph_size': LaunchConfiguration('morph_size'),
            'min_area':   LaunchConfiguration('min_area'),
            'debug':      LaunchConfiguration('debug'),
            'search_timeout':  LaunchConfiguration('search_timeout'),
            'stop_timeout':    LaunchConfiguration('stop_timeout'),
        }],
    )

    controller_node = Node(
        package    = 'controller',
        executable = 'controller_node',
        name       = 'controller_node',
        output     = 'screen',
        parameters = [{
            'kp':               LaunchConfiguration('kp'),
            'base_speed':       LaunchConfiguration('base_speed'),
            'max_angular':      LaunchConfiguration('max_angular'),
            'min_speed':        LaunchConfiguration('min_speed'),
            'speed_reduction':  LaunchConfiguration('speed_reduction'),
            'search_angular':   LaunchConfiguration('search_angular'),
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
            'max_speed':        LaunchConfiguration('motor_max_speed'),
            'steer_gain':       LaunchConfiguration('steer_gain'),
            'watchdog_period':  LaunchConfiguration('motor_watchdog_period'),
            'watchdog_timeout': LaunchConfiguration('motor_watchdog_timeout'),
        }],
    )

    return LaunchDescription([
        # Caméra
        arg_camera_device, arg_pixel_format, arg_output_encoding,
        # Vision
        arg_roi_ratio, arg_use_otsu, arg_threshold,
        arg_blur_size, arg_morph_size, arg_min_area, arg_debug, 
        arg_search_timeout, arg_stop_timeout,
        # Contrôleur
        arg_kp, arg_base_speed, arg_max_angular, arg_min_speed,
        arg_speed_reduction, arg_search_angular,
        arg_ctrl_watchdog_period, arg_ctrl_watchdog_timeout,
        # Moteur
        arg_motor_max_speed, arg_steer_gain,
        arg_motor_watchdog_period, arg_motor_watchdog_timeout,
        # Nœuds
        v4l2_camera_node,
        camera_node,
        controller_node,
        motor_node,
    ])
