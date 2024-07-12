## Directory info

- `genesis`: Contains systemd service file to automatically start ROS2 ecosystem on rpi4 device. It is **not** a ROS2 package.

- `sync_rpi4`: Contains scripts for exchanging files between rpi4 device and current PC used. It is **not** a ROS2 package.

- `road_quality_core`: Contains core node that handles miscellaneous jobs in rpi4 device.

- `road_quality_gpio`: Handles rpi4 device GPIO pins.

- `road_quality_icm20948_cpp`: Handles communicatino with IMU sensor (ICM20948) through I2C protocol.

- `road_quality_mtk3339_cpp`: Handles communicatino with GPS sensor (MTK3339) through a serial port.

- `road_quality_msgs`: Contains custom ROS2 messages and services definitions.

- `road_quality_monitor`: Node with UI environment used for monitoring live data broadcasted from rpi4 device. Has to be used from external PC connected to same network as rpi4 device.

- `road_quality_calib`: Node that performs IMU calibration.
