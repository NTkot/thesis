import rclpy
import sys
from monitor import Monitor
from PyQt5 import QtCore, QtWidgets
import ament_index_python
import yaml

def main(args=None):
    rclpy.init(args=args)

    with open(ament_index_python.get_package_share_directory('road_quality_monitor') + '/monitor.yaml') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)

    app = QtWidgets.QApplication(sys.argv)

    monitor_node = Monitor()

    def spin_ros():
        rclpy.spin_once(monitor_node, timeout_sec=0.1)

    timer = QtCore.QTimer()
    timer.timeout.connect(spin_ros)
    timer.start(int(data['spin_ros_period_ms']))
    app.exec_()

    monitor_node.destroy_node()
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()
