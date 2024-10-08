import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    #get_package_share_directory(package_name)
    declare_print_arg = DeclareLaunchArgument(
        'print',
        default_value='false',
        description='Enable printing'
    )
    print_param = LaunchConfiguration('print')
    
    declare_calib_data_file_arg = DeclareLaunchArgument(
        'calib_data_file',
        default_value=os.path.join(get_package_prefix('mpu9250'), 'config', 'calib_data.json'),
        description='Path to the calibration data file'
    )
    calib_data_file_param = LaunchConfiguration('calib_data_file')

    declare_calibrate_arg = DeclareLaunchArgument(
        'calibrate',
        default_value='false',
        description='Do Gyro and Magnetometer calibration'
    )
    calibrate_param = LaunchConfiguration('calibrate')

    return LaunchDescription([
        declare_print_arg,
        declare_calibrate_arg,
        declare_calib_data_file_arg,
        
        Node(
            package="mpu9250",
            executable="mpu9250",
            name="mpu9250",
            parameters=[
                {"acceleration_scale": [1.0070642813137283, 1.0077919346121842, 0.979079278290781], 
                 "acceleration_bias": [0.30351858720785296, 0.03640315588498285, 0.014441728200428484],
                 "print": print_param,
                 "calibrate": calibrate_param,
                 "calib_data_file": calib_data_file_param,
                 "do_apply_ema": True,
                 "ema_alpha": 0.1}
            ],
        )
    ])
