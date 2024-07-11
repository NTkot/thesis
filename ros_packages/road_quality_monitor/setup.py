from setuptools import setup

package_name = 'road_quality_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'config/monitor.yaml',
                                   'resources/road_icon.png',
                                   'resources/red_led_on.png',
                                   'resources/red_led_off.png']),
        ('lib/' + package_name, ['road_quality_monitor/monitor.py',
                                 'road_quality_monitor/main.py',
                                 'ui/interface.py',
                                 'ui/analog_gauge_widget.py',
                                 'ui/compasswidget.py',
                                 'road_quality_monitor/utils/buffers.py',
                                 'road_quality_monitor/utils/plot_utils.py',
                                 'road_quality_monitor/parsers/imu_parsers.py',
                                 'road_quality_monitor/parsers/gps_parsers.py'])
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
            'monitor_gui = road_quality_monitor.main:main'
        ],
    },
)
