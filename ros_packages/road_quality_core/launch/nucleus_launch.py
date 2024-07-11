import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    rosbag_on_button_enable_arg = DeclareLaunchArgument(
        'rosbag_on_button_enable', default_value=TextSubstitution(text='true')
    )
    rosbag_on_button_input_pin_arg = DeclareLaunchArgument(
        'rosbag_on_button_input_pin', default_value=TextSubstitution(text='25')
    )
    rosbag_on_button_led_pin_arg = DeclareLaunchArgument(
        'rosbag_on_button_led_pin', default_value=TextSubstitution(text='26')
    )
    rosbag_on_button_output_path_arg = DeclareLaunchArgument(
        'rosbag_on_button_output_path', default_value=TextSubstitution(text='/mnt/usb/bag_db')
    )

    imu_static_calib_on_button_enable_arg = DeclareLaunchArgument(
        'imu_static_calib_on_button_enable', default_value=TextSubstitution(text='true')
    )
    imu_static_calib_on_button_input_pin_arg = DeclareLaunchArgument(
        'imu_static_calib_on_button_input_pin', default_value=TextSubstitution(text='16')
    )
    imu_static_calib_on_button_led_pin_arg = DeclareLaunchArgument(
        'imu_static_calib_on_button_led_pin', default_value=TextSubstitution(text='19')
    )

    imu_dyn_calib_on_button_enable_arg = DeclareLaunchArgument(
        'imu_dyn_calib_on_button_enable', default_value=TextSubstitution(text='true')
    )
    imu_dyn_calib_on_button_input_pin_arg = DeclareLaunchArgument(
        'imu_dyn_calib_on_button_input_pin', default_value=TextSubstitution(text='20')
    )
    imu_dyn_calib_on_button_led_pin_arg = DeclareLaunchArgument(
        'imu_dyn_calib_on_button_led_pin', default_value=TextSubstitution(text='6')
    )

    ros_args_list = [
        rosbag_on_button_enable_arg,
        rosbag_on_button_input_pin_arg,
        rosbag_on_button_led_pin_arg,
        rosbag_on_button_output_path_arg,
        imu_static_calib_on_button_enable_arg,
        imu_static_calib_on_button_input_pin_arg,
        imu_static_calib_on_button_led_pin_arg,
        imu_dyn_calib_on_button_enable_arg,
        imu_dyn_calib_on_button_input_pin_arg,
        imu_dyn_calib_on_button_led_pin_arg
    ]

    ros_nodes_list = [
        Node(
            package='road_quality_gpio',
            executable='gpio_handler',
            name='gpio_handler'
        ),
        Node(
            package='road_quality_mtk3339_cpp',
            executable='mtk3339_cpp_reader',
            name='mtk3339_cpp_reader'
        ),
        Node(
            package='road_quality_icm20948_cpp',
            executable='icm20948_cpp_reader',
            name='icm20948_reader'
        ),
        Node(
            package='road_quality_core',
            executable='nucleus',
            name='nucleus',
            parameters=[{
                'rosbag_on_button_enable'              : LaunchConfiguration('rosbag_on_button_enable'),
                'rosbag_on_button_input_pin'           : LaunchConfiguration('rosbag_on_button_input_pin'),
                'rosbag_on_button_led_pin'             : LaunchConfiguration('rosbag_on_button_led_pin'),
                'rosbag_on_button_output_path'         : LaunchConfiguration('rosbag_on_button_output_path'),
                'imu_static_calib_on_button_enable'    : LaunchConfiguration('imu_static_calib_on_button_enable'),
                'imu_static_calib_on_button_input_pin' : LaunchConfiguration('imu_static_calib_on_button_input_pin'),
                'imu_static_calib_on_button_led_pin'   : LaunchConfiguration('imu_static_calib_on_button_led_pin'),
                'imu_dyn_calib_on_button_enable'       : LaunchConfiguration('imu_dyn_calib_on_button_enable'),
                'imu_dyn_calib_on_button_input_pin'    : LaunchConfiguration('imu_dyn_calib_on_button_input_pin'),
                'imu_dyn_calib_on_button_led_pin'      : LaunchConfiguration('imu_dyn_calib_on_button_led_pin'),
            }]
        )
    ]

    if os.path.exists('/dev/video0'):
        video_device_launch_arg = DeclareLaunchArgument(
            'video_device', default_value=TextSubstitution(text='/dev/video0')
        )
        image_size_launch_arg = DeclareLaunchArgument(
            'image_size', default_value=TextSubstitution(text='[640, 480]')
        )

        return LaunchDescription([
            *ros_args_list,
            video_device_launch_arg,
            image_size_launch_arg,
            *ros_nodes_list,
            Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                name='camera_record',
                parameters=[{
                    'video_device' : LaunchConfiguration('video_device'),
                    'image_size'   : LaunchConfiguration('image_size'),
                }]
            )
        ])

    else:
        return LaunchDescription([
            *ros_args_list,
            *ros_nodes_list
        ])
