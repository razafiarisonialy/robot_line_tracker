from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # v4l2_camera_node
    arg_camera_device   = DeclareLaunchArgument('camera_device',   default_value='/dev/video0', description='Périphérique caméra USB (ex: /dev/video0)')
    arg_pixel_format    = DeclareLaunchArgument('pixel_format',    default_value='YUYV',        description='Format pixel V4L2 : YUYV, UYVY ou GREY')
    arg_output_encoding = DeclareLaunchArgument('output_encoding', default_value='bgr8',        description='Encodage sortie cv_bridge — bgr8 requis par cameraNode')

    #camera_node
    arg_roi_ratio   = DeclareLaunchArgument('roi_ratio',   default_value='0.5',  description='Fraction ROI depuis le bas (0.0–1.0)')
    arg_use_otsu    = DeclareLaunchArgument('use_otsu',    default_value='True', description='Seuillage adaptatif Otsu')
    arg_threshold   = DeclareLaunchArgument('threshold',   default_value='80',   description='Seuil fixe si use_otsu=False')
    arg_min_area    = DeclareLaunchArgument('min_area',    default_value='300',  description='Surface minimale de contour (px²)')
    arg_blur_size   = DeclareLaunchArgument('blur_size',   default_value='7',    description='Taille filtre gaussien (impair)')
    arg_morph_size  = DeclareLaunchArgument('morph_size',  default_value='5',    description='Taille noyau morphologique')
    arg_debug       = DeclareLaunchArgument('debug',       default_value='True', description='Publier images de debug')

    #controller_node
    arg_kp              = DeclareLaunchArgument('kp',              default_value='0.003', description='Gain proportionnel Kp')
    arg_base_speed      = DeclareLaunchArgument('base_speed',      default_value='0.15',  description='Vitesse linéaire de croisière du controller (m/s)')
    arg_max_angular     = DeclareLaunchArgument('max_angular',     default_value='0.8',   description='Saturation angulaire (rad/s)')
    arg_speed_reduction = DeclareLaunchArgument('speed_reduction', default_value='0.3',   description='Coefficient réduction vitesse en virage')
    arg_min_speed       = DeclareLaunchArgument('min_speed',       default_value='0.05',  description='Vitesse linéaire minimale (m/s)')

    # motor_node
    arg_motor_base_speed = DeclareLaunchArgument('motor_base_speed', default_value='200',  description='Vitesse de base moteurs GoPiGo3 (DPS) → motor_node.base_speed')
    arg_motor_max_speed  = DeclareLaunchArgument('motor_max_speed',  default_value='400',  description='Vitesse maximale moteurs GoPiGo3 (DPS) → motor_node.max_speed')
    arg_steer_gain       = DeclareLaunchArgument('steer_gain',       default_value='50.0', description='Gain de braquage en DPS/(rad/s) — 0.0 = cinématique différentielle')

    # Nœud: v4l2_camera_node
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

    # Nœud: camera_node
    camera_node = Node(
        package    = 'camera',
        executable = 'camera_node',
        name       = 'camera_node',
        output     = 'screen',
        parameters = [{
            'roi_ratio':  LaunchConfiguration('roi_ratio'),
            'use_otsu':   LaunchConfiguration('use_otsu'),
            'threshold':  LaunchConfiguration('threshold'),
            'min_area':   LaunchConfiguration('min_area'),
            'blur_size':  LaunchConfiguration('blur_size'),
            'morph_size': LaunchConfiguration('morph_size'),
            'debug':      LaunchConfiguration('debug'),
        }],
    )

    # Nœud : controller_node
    controller_node = Node(
        package    = 'controller',
        executable = 'controller_node',
        name       = 'controller_node',
        output     = 'screen',
        parameters = [{
            'kp':              LaunchConfiguration('kp'),
            'base_speed':      LaunchConfiguration('base_speed'),
            'max_angular':     LaunchConfiguration('max_angular'),
            'speed_reduction': LaunchConfiguration('speed_reduction'),
            'min_speed':       LaunchConfiguration('min_speed'),
        }],
    )

    # Nœud : motor_node
    motor_node = Node(
        package    = 'moteur',
        executable = 'motor_node',
        name       = 'motor_node',
        output     = 'screen',
        parameters = [{
            'base_speed': LaunchConfiguration('motor_base_speed'),  # DPS
            'max_speed':  LaunchConfiguration('motor_max_speed'),   # DPS
            'steer_gain': LaunchConfiguration('steer_gain'),
        }],
    )

    return LaunchDescription([
        # v4l2
        arg_camera_device,
        arg_pixel_format,
        arg_output_encoding,
        # camera
        arg_roi_ratio,
        arg_use_otsu,
        arg_threshold,
        arg_min_area,
        arg_blur_size,
        arg_morph_size,
        arg_debug,
        # controller
        arg_kp,
        arg_base_speed,
        arg_max_angular,
        arg_speed_reduction,
        arg_min_speed,
        # motor
        arg_motor_base_speed,
        arg_motor_max_speed,
        arg_steer_gain,
        # nœuds
        v4l2_camera_node,
        camera_node,
        controller_node,
        motor_node,
    ])