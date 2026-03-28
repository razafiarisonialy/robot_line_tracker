from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # v4l2_camera_node
    arg_camera_device   = DeclareLaunchArgument('camera_device',   default_value='/dev/video0')
    arg_pixel_format    = DeclareLaunchArgument('pixel_format',    default_value='YUYV')
    arg_output_encoding = DeclareLaunchArgument('output_encoding', default_value='bgr8')

    # camera_node
    arg_roi_ratio      = DeclareLaunchArgument('roi_ratio',      default_value='0.30',  description='CORRECTION: réduit de 0.5 → 0.30 pour ne voir que le sol proche')
    arg_use_otsu       = DeclareLaunchArgument('use_otsu',       default_value='True')
    arg_threshold      = DeclareLaunchArgument('threshold',      default_value='80')
    arg_min_area       = DeclareLaunchArgument('min_area',       default_value='300')
    arg_blur_size      = DeclareLaunchArgument('blur_size',      default_value='7')
    arg_morph_size     = DeclareLaunchArgument('morph_size',     default_value='5')
    arg_debug          = DeclareLaunchArgument('debug',          default_value='True')
    arg_search_timeout = DeclareLaunchArgument('search_timeout', default_value='1.5',  description='CORRECTION: réduit 3.0→1.5 pour réagir plus vite')
    arg_stop_timeout   = DeclareLaunchArgument('stop_timeout',   default_value='4.0',  description='CORRECTION: réduit 6.0→4.0')
    arg_use_multi_roi  = DeclareLaunchArgument('use_multi_roi',  default_value='True',  description='NOUVEAU: détection multi-niveaux pour virages')
    arg_roi_levels     = DeclareLaunchArgument('roi_levels',     default_value='3',     description='NOUVEAU: nombre de niveaux ROI (1=désactivé)')

    # controller_node
    arg_kp              = DeclareLaunchArgument('kp',              default_value='0.008',  description='CORRECTION: augmenté pour réagir vite (0.003→0.008)')
    arg_base_speed      = DeclareLaunchArgument('base_speed',      default_value='0.08',   description='CORRECTION: réduit pour les virages (0.15→0.08 m/s)')
    arg_max_angular     = DeclareLaunchArgument('max_angular',     default_value='1.0',    description='Légèrement augmenté pour couvrir les virages U')
    arg_speed_reduction = DeclareLaunchArgument('speed_reduction', default_value='0.5',    description='CORRECTION: plus agressif en virage (0.3→0.5)')
    arg_min_speed       = DeclareLaunchArgument('min_speed',       default_value='0.04')
    arg_search_angular  = DeclareLaunchArgument('search_angular',  default_value='0.5',    description='CORRECTION: augmenté pour couvrir les virages U (0.4→0.5)')
    arg_curve_threshold = DeclareLaunchArgument('curve_threshold', default_value='0.4',    description='CORRECTION: seuil de boost plus bas (0.5→0.4)')
    arg_curve_boost     = DeclareLaunchArgument('curve_boost',     default_value='1.6',    description='CORRECTION: boost plus fort (1.3→1.6)')
    arg_search_ramp_time= DeclareLaunchArgument('search_ramp_time',default_value='1.0',    description='NOUVEAU: rampe de vitesse en recherche (s)')

    # motor_node
    arg_motor_base_speed = DeclareLaunchArgument('motor_base_speed', default_value='200')
    arg_motor_max_speed  = DeclareLaunchArgument('motor_max_speed',  default_value='400')
    arg_steer_gain       = DeclareLaunchArgument('steer_gain',       default_value='50.0')

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
            'roi_ratio':       LaunchConfiguration('roi_ratio'),
            'use_otsu':        LaunchConfiguration('use_otsu'),
            'threshold':       LaunchConfiguration('threshold'),
            'min_area':        LaunchConfiguration('min_area'),
            'blur_size':       LaunchConfiguration('blur_size'),
            'morph_size':      LaunchConfiguration('morph_size'),
            'debug':           LaunchConfiguration('debug'),
            'search_timeout':  LaunchConfiguration('search_timeout'),
            'stop_timeout':    LaunchConfiguration('stop_timeout'),
            'use_multi_roi':   LaunchConfiguration('use_multi_roi'),
            'roi_levels':      LaunchConfiguration('roi_levels'),
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
            'speed_reduction':  LaunchConfiguration('speed_reduction'),
            'min_speed':        LaunchConfiguration('min_speed'),
            'search_angular':   LaunchConfiguration('search_angular'),
            'curve_threshold':  LaunchConfiguration('curve_threshold'),
            'curve_boost':      LaunchConfiguration('curve_boost'),
            'search_ramp_time': LaunchConfiguration('search_ramp_time'),
        }],
    )

    motor_node = Node(
        package    = 'moteur',
        executable = 'motor_node',
        name       = 'motor_node',
        output     = 'screen',
        parameters = [{
            'base_speed': LaunchConfiguration('motor_base_speed'),
            'max_speed':  LaunchConfiguration('motor_max_speed'),
            'steer_gain': LaunchConfiguration('steer_gain'),
        }],
    )

    return LaunchDescription([
        arg_camera_device, arg_pixel_format, arg_output_encoding,
        arg_roi_ratio, arg_use_otsu, arg_threshold, arg_min_area,
        arg_blur_size, arg_morph_size, arg_debug,
        arg_search_timeout, arg_stop_timeout,
        arg_use_multi_roi, arg_roi_levels,
        arg_kp, arg_base_speed, arg_max_angular, arg_speed_reduction,
        arg_min_speed, arg_search_angular,
        arg_curve_threshold, arg_curve_boost, arg_search_ramp_time,
        arg_motor_base_speed, arg_motor_max_speed, arg_steer_gain,
        v4l2_camera_node,
        camera_node,
        controller_node,
        motor_node,
    ])