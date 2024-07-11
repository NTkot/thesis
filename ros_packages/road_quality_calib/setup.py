from setuptools import setup

package_name = 'road_quality_calib'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name,   ['road_quality_calib/imu_static_calib.py',
                                   'road_quality_calib/imu_dyn_calib.py',
                                   'road_quality_calib/calibration.py',
                                   'road_quality_calib/calib_utils.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ntkot',
    maintainer_email='kotarelasn@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_static_calib = road_quality_calib.calibration:static_calib_main',
            'imu_static_calib_reverse_parking = road_quality_calib.calibration:static_calib_rp_main',
            'imu_static_calib_user_values = road_quality_calib.calibration:static_calib_uv_main',
            'imu_dyn_calib_magn = road_quality_calib.calibration:dyn_calib_magn_main',
            'imu_dyn_calib_brake = road_quality_calib.calibration:dyn_calib_brake_main'
            'imu_dyn_calib_user_values = road_quality_calib.calibration:dyn_calib_uv_main',
        ],
    },
)
