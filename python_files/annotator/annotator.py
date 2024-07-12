from PyQt5 import QtWidgets, QtGui, QtCore

from ui.annotator_ui import Ui_main_window
from ui.image_window import ImageWindow
from annotator_utils import ImuTopicBuffer, saturate, binary_search, annotation_text, annotation_tooltip

import os
import sys
import yaml
import time
import warnings
import pyqtgraph as pg
from data_process.utils import get_by_path, CompressPickleWrapper
from data_process.signal_tools import signal_filter
from data_process.labels import AnnotationList, Annotation, Label, AnomalyLabel, TransversityLabel, SeverityLabel


class Annotator():
    def __init__(self, pickle_filepath: str):
        self.pickle_filepath = pickle_filepath

        self.load_config()

        self.load_data()

        self.init_gui()

        self.init_vars()

        self.setup_plot_widgets()

        self.setup_ext_button_plot_widget()

        self.setup_annotate_ui()

        self.setup_settings_tab()

        self.setup_slider_ui()

        self.setup_hotkeys()

        if not sys.platform.startswith('linux'):
            print("Detected Windows platform, setting appropriate settings...")
            self.setup_win()


    def load_config(self):
        with open(os.path.join(os.path.dirname(__file__), 'annotator.yaml')) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
        

    def load_data(self):
        if not os.path.exists(self.pickle_filepath):
            raise ValueError(f"Provided pickle file does not exist in filesystem ({self.pickle_filepath})")
        if not os.path.isfile(self.pickle_filepath):
            raise ValueError(f"Provided pickle file does not point to a file ({self.pickle_filepath})")
        self.data = CompressPickleWrapper.load(self.pickle_filepath)

        self.imu_data : dict[str, ImuTopicBuffer] = {}
        self.ext_button_data = {}
        for key in self.data.keys():
            if (key == '/imu/raw') and (not '/imu/raw' in self.imu_data.keys()):
                self.imu_data['/imu/raw'] = ImuTopicBuffer(abs_time=get_by_path(self.data, ['/imu/raw', 'time_ns']),
                                                           rel_time=get_by_path(self.data, ['/imu/raw', 'time']),
                                                           a_x=get_by_path(self.data, ['/imu/raw', 'linear_acceleration', 'x']),
                                                           a_y=get_by_path(self.data, ['/imu/raw', 'linear_acceleration', 'y']),
                                                           a_z=get_by_path(self.data, ['/imu/raw', 'linear_acceleration', 'z']),
                                                           g_x=get_by_path(self.data, ['/imu/raw', 'angular_velocity', 'x']),
                                                           g_y=get_by_path(self.data, ['/imu/raw', 'angular_velocity', 'y']),
                                                           g_z=get_by_path(self.data, ['/imu/raw', 'angular_velocity', 'z']))
                self.imu_data['/imu/raw'].apply_filter(lambda x: signal_filter.wiener(x, self.config['wiener_filter_size']))
                self.imu_data['/imu/raw'].apply_filter(lambda x: signal_filter.moving_average(x, self.config['moving_average_size']))
                continue
            if (key == '/imu/static_calib') and (not '/imu/static_calib' in self.imu_data.keys()):
                self.imu_data['/imu/static_calib'] = ImuTopicBuffer(abs_time=get_by_path(self.data, ['/imu/static_calib', 'time_ns']),
                                                                    rel_time=get_by_path(self.data, ['/imu/static_calib', 'time']),
                                                                    a_x=get_by_path(self.data, ['/imu/static_calib', 'linear_acceleration', 'x']),
                                                                    a_y=get_by_path(self.data, ['/imu/static_calib', 'linear_acceleration', 'y']),
                                                                    a_z=get_by_path(self.data, ['/imu/static_calib', 'linear_acceleration', 'z']),
                                                                    g_x=get_by_path(self.data, ['/imu/static_calib', 'angular_velocity', 'x']),
                                                                    g_y=get_by_path(self.data, ['/imu/static_calib', 'angular_velocity', 'y']),
                                                                    g_z=get_by_path(self.data, ['/imu/static_calib', 'angular_velocity', 'z']))
                self.imu_data['/imu/static_calib'].apply_filter(lambda x: signal_filter.wiener(x, self.config['wiener_filter_size']))
                self.imu_data['/imu/static_calib'].apply_filter(lambda x: signal_filter.moving_average(x, self.config['moving_average_size']))
                continue
            if (key == '/imu/calib') and (not '/imu/calib' in self.imu_data.keys()):
                self.imu_data['/imu/calib'] = ImuTopicBuffer(abs_time=get_by_path(self.data, ['/imu/calib', 'time_ns']),
                                                             rel_time=get_by_path(self.data, ['/imu/calib', 'time']),
                                                             a_x=get_by_path(self.data, ['/imu/calib', 'linear_acceleration', 'x']),
                                                             a_y=get_by_path(self.data, ['/imu/calib', 'linear_acceleration', 'y']),
                                                             a_z=get_by_path(self.data, ['/imu/calib', 'linear_acceleration', 'z']),
                                                             g_x=get_by_path(self.data, ['/imu/calib', 'angular_velocity', 'x']),
                                                             g_y=get_by_path(self.data, ['/imu/calib', 'angular_velocity', 'y']),
                                                             g_z=get_by_path(self.data, ['/imu/calib', 'angular_velocity', 'z']))
                self.imu_data['/imu/calib'].apply_filter(lambda x: signal_filter.wiener(x, self.config['wiener_filter_size']))
                self.imu_data['/imu/calib'].apply_filter(lambda x: signal_filter.moving_average(x, self.config['moving_average_size']))
                continue
            if(key == '/gpio/inputs/pin23') and (not 'state' in self.ext_button_data.keys()):
                self.ext_button_data['time']  = get_by_path(self.data, ['/gpio/inputs/pin23', 'time'])
                self.ext_button_data['state'] = get_by_path(self.data, ['/gpio/inputs/pin23', 'state'])
                continue
        
        self._find_max_min_abs_time()


    def _find_max_min_abs_time(self):
        tree_time_elements = [[elem, 'time_ns'] for elem in self.data.keys() if 'time_ns' in self.data[elem].keys()]

        time_start_list = []
        time_end_list = []
        for time_element in tree_time_elements:
            time_start_list.append(get_by_path(self.data, time_element)[0])
            time_end_list.append(get_by_path(self.data, time_element)[-1])
        self.minimum_time: int = min(time_start_list)
        self.maximum_time: int = max(time_end_list)


    def init_gui(self):
        self.main_window = QtWidgets.QMainWindow()
        self.ui = Ui_main_window()
        self.ui.setupUi(self.main_window)
        self.main_window.show()
        self.main_window.setWindowIcon(QtGui.QIcon(os.path.join(os.path.dirname(__file__), 
                                                                'ui', 
                                                                'annotator_icon.png')))


    def init_vars(self):
        self.last_cam_rel_time = 0

        self.annotations = AnnotationList()
        self.annotation_regions_acc : list[pg.LinearRegionItem] = []
        self.annotation_regions_gyr : list[pg.LinearRegionItem] = []

        self.ann_start_line_offset = 1
        self.ann_end_line_offset   = 1

        self.max_rel_time = max([imu_data.rel_time[-1] for imu_data in self.imu_data.values()])
        self.min_t_slider = 0
        self.max_t_slider = 2147483647

        self.camera_times = get_by_path(self.data, ['/image_raw/compressed', 'time_ns'])
        self.cam_last_idx = -1

        self.time_play_button_timer = QtCore.QTimer()
        self.time_play_button_timer.timeout.connect(self._time_play_button_timer_callback)

        self.enlarged_camera_view = False


    def setup_settings_tab(self):
        if not self.imu_data.keys():
            raise ValueError('No IMU records found')
        else:
            self.ui.imu_topic_dropdown.addItems(self.imu_data.keys())
            self.ui.imu_topic_dropdown.activated[str].connect(self._set_imu_topic_on_plots)
            if '/imu/calib' in self.imu_data.keys():
                self._set_imu_topic_on_plots('/imu/calib')
                text_selected = '/imu/calib'
            elif '/imu/static_calib' in self.imu_data.keys():
                self._set_imu_topic_on_plots('/imu/static_calib')
                text_selected = '/imu/static_calib'
            else:
                self._set_imu_topic_on_plots('/imu/raw')
                text_selected = '/imu/raw'

        index = self.ui.imu_topic_dropdown.findText(text_selected)
        self.ui.imu_topic_dropdown.setCurrentIndex(index)

        self.ui.wiener_size_box.setKeyboardTracking(False)
        self.ui.wiener_size_box.setValue(self.config['wiener_filter_size'])
        self.ui.wiener_size_box.valueChanged.connect(self._filter_settings_changed)

        self.ui.mv_size_box.setKeyboardTracking(False)
        self.ui.mv_size_box.setValue(self.config['moving_average_size'])
        self.ui.mv_size_box.valueChanged.connect(self._filter_settings_changed)
        
        self.ui.time_disp_box.setKeyboardTracking(False)
        self.ui.time_disp_box.setValue(self.config['time_displayed_sec'])
        self.ui.time_disp_box.valueChanged.connect(self._time_displayed_changed)

        self.ui.prev_t_jump_box.setKeyboardTracking(False)
        self.ui.prev_t_jump_box.setValue(self.config['previous_time_jump_sec'])
        self.ui.prev_t_jump_box.valueChanged.connect(self._control_settings_changed)

        self.ui.next_t_jump_box.setKeyboardTracking(False)
        self.ui.next_t_jump_box.setValue(self.config['previous_time_jump_sec'])
        self.ui.next_t_jump_box.valueChanged.connect(self._control_settings_changed)


    def setup_annotate_ui(self):
        self.ui.t_start_box.setKeyboardTracking(False)
        self.ui.t_end_box.setKeyboardTracking(False)
        self.ui.t_start_box.valueChanged.connect(self._ann_time_start_box_changed)
        self.ui.t_end_box.valueChanged.connect(self._ann_time_end_box_changed)

        self.ui.t_start_box.setMaximum(self.max_rel_time)
        self.ui.t_end_box.setMaximum(self.max_rel_time)

        self.ui.anomaly_dropdown.addItems([label.name for label in AnomalyLabel])
        self.ui.transversity_dropdown.addItems([label.name for label in TransversityLabel])
        self.ui.severity_dropdown.addItems([label.name for label in SeverityLabel])

        self.ui.add_button.clicked.connect(self._insert_annotation)
        self.ui.remove_button.clicked.connect(self._remove_annotation)

        font = QtGui.QFont("monospace", 10)
        self.ui.annotation_list.setFont(font)
        self.ui.annotation_list.itemDoubleClicked.connect(self._clicked_annotation)
        self.ui.annotation_list.itemChanged.connect(self._changed_annotation_list)

        self.ui.dump_button.clicked.connect(self._dump_annotations)
        self.ui.load_button.clicked.connect(self._load_annotations)

        self.ui.camera_view.mouseDoubleClickEvent = self.image_popup


    def setup_plot_widgets(self):
        self.ui.acc_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.acc_x_plot : pg.PlotDataItem = self.ui.acc_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.acc_y_plot : pg.PlotDataItem = self.ui.acc_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.acc_z_plot : pg.PlotDataItem = self.ui.acc_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.acc_plot.setBackground((34, 39, 40))
        self.ui.acc_plot.setLabel('left', 'm/s^2')
        self.ui.acc_plot.setLabel('bottom', 'sec')
        self.ui.acc_plot.showGrid(x=True, y=True, alpha=0.5)

        self.ui.gyr_plot.addLegend(offset=(1, 1), verSpacing=-7.5)
        self.gyr_x_plot : pg.PlotDataItem = self.ui.gyr_plot.plot(pen=pg.mkPen('r', width=1), name='X')
        self.gyr_y_plot : pg.PlotDataItem = self.ui.gyr_plot.plot(pen=pg.mkPen('b', width=1), name='Y')
        self.gyr_z_plot : pg.PlotDataItem = self.ui.gyr_plot.plot(pen=pg.mkPen('g', width=1), name='Z')
        self.ui.gyr_plot.setBackground((34, 39, 40))
        self.ui.gyr_plot.setLabel('left', 'rad/s')
        self.ui.gyr_plot.setLabel('bottom', 'sec')
        self.ui.gyr_plot.showGrid(x=True, y=True, alpha=0.5)

        self.acc_plot_centerline = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('w', width=1, style=QtCore.Qt.SolidLine))
        self.gyr_plot_centerline = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('w', width=1, style=QtCore.Qt.SolidLine))
        self.ui.acc_plot.addItem(self.acc_plot_centerline)
        self.ui.gyr_plot.addItem(self.gyr_plot_centerline)
        self.acc_plot_centerline.setValue(0)
        self.gyr_plot_centerline.setValue(0)

        self.acc_plot_start_line = pg.InfiniteLine(angle=90, movable=True,  pen=pg.mkPen(color=QtGui.QColor(0, 157, 0), 
                                                                                         width=1, 
                                                                                         style=QtCore.Qt.SolidLine))
        self.acc_plot_end_line   = pg.InfiniteLine(angle=90, movable=True,  pen=pg.mkPen(color='r', 
                                                                                         width=1, 
                                                                                         style=QtCore.Qt.SolidLine))
        self.gyr_plot_start_line = pg.InfiniteLine(angle=90, movable=True,  pen=pg.mkPen(color=QtGui.QColor(0, 157, 0), 
                                                                                         width=1, 
                                                                                         style=QtCore.Qt.SolidLine))
        self.gyr_plot_end_line   = pg.InfiniteLine(angle=90, movable=True,  pen=pg.mkPen(color='r', 
                                                                                         width=1, 
                                                                                         style=QtCore.Qt.SolidLine))
        self.ui.acc_plot.addItem(self.acc_plot_start_line)
        self.ui.acc_plot.addItem(self.acc_plot_end_line)
        self.ui.gyr_plot.addItem(self.gyr_plot_start_line)
        self.ui.gyr_plot.addItem(self.gyr_plot_end_line)
        self.acc_plot_start_line.sigDragged.connect(self._ann_start_dragged)
        self.gyr_plot_start_line.sigDragged.connect(self._ann_start_dragged)
        self.acc_plot_end_line.sigDragged.connect(self._ann_end_dragged)
        self.gyr_plot_end_line.sigDragged.connect(self._ann_end_dragged)
        self.acc_plot_start_line.sigPositionChangeFinished.connect(self._ann_start_pos_changed)
        self.gyr_plot_start_line.sigPositionChangeFinished.connect(self._ann_start_pos_changed)
        self.acc_plot_end_line.sigPositionChangeFinished.connect(self._ann_end_pos_changed)
        self.gyr_plot_end_line.sigPositionChangeFinished.connect(self._ann_end_pos_changed)

        self.ui.acc_plot.setXLink(self.ui.gyr_plot)


    def setup_ext_button_plot_widget(self):
        self.ext_button_plot_item : pg.PlotDataItem = self.ui.ext_button_plot.plot(pen=pg.mkPen([204, 119, 34, 100], width=1))
        self.ui.ext_button_plot.setBackground((34, 39, 40))
        self.ui.ext_button_plot.setLabel('bottom', 'sec')
        self.ui.ext_button_plot.setLabel('left', 'state')

        if self.ext_button_data:
            self.ext_button_plot_item.setData(self.ext_button_data['time'], self.ext_button_data['state'])

        self.ui.ext_button_plot.setYRange(-0.1, 1.1)
        self.ui.ext_button_plot.getAxis('left').setTicks([[(state, str(state)) for state in [0, 1]]])
        
        self.ui.ext_button_plot.setXLink(self.ui.acc_plot)


    def setup_slider_ui(self):
        self.ui.time_slider.setMinimum(self.min_t_slider)
        self.ui.time_slider.setMaximum(self.max_t_slider)
        self.ui.time_slider.sliderMoved.connect(self._slider_changed)
        self.ui.time_slider.sliderReleased.connect(self._slider_changed)

        self.ui.time_box.setKeyboardTracking(False)
        self.ui.time_box.setMinimum(0.0)
        self.ui.time_box.setMaximum(self.max_rel_time)
        self.ui.time_box.valueChanged.connect(self._time_box_changed)

        self.ui.time_play_button.setIcon(QtGui.QIcon(os.path.join(os.path.dirname(__file__), 'ui', 'play.png')))
        self.ui.time_play_button.clicked.connect(self._time_play_button_pressed)


    def setup_hotkeys(self):
        self.ui.dump_annotations.triggered.connect(self._dump_annotations)
        self.ui.load_annotations.triggered.connect(self._load_annotations)
        self.ui.play_pause.triggered.connect(self.ui.time_play_button.click)
        self.ui.prev_t.triggered.connect(lambda: self.ui.time_box.setValue(self.ui.time_box.value() - self.config['previous_time_jump_sec']))
        self.ui.next_t.triggered.connect(lambda: self.ui.time_box.setValue(self.ui.time_box.value() + self.config['next_time_jump_sec']))

        self.ui.main_widget.addActions([self.ui.dump_annotations,
                                        self.ui.load_annotations,
                                        self.ui.play_pause,
                                        self.ui.prev_t,
                                        self.ui.next_t])


    def image_popup(self, event):
        self.image_window = ImageWindow(qimage=
            QtGui.QImage.fromData(get_by_path(self.data, ['/image_raw/compressed', 'data', self.cam_last_idx])))
        self.image_window.show()


    def setup_win(self):
        tabWidgetFont = QtGui.QFont()
        tabWidgetFont.setPointSize(10)
        tabWidgetFont.setBold(False)
        tabWidgetFont.setItalic(False)
        tabWidgetFont.setWeight(50)
        self.ui.tabWidget.setFont(tabWidgetFont)

        self.ui.tabWidget.setStyleSheet("QTabWidget::pane {\n"
                                        "    border: 1px solid black;\n"
                                        "    background: rgb(76, 86, 90);\n"
                                        "}\n"
                                        "\n"
                                        "QTabWidget::tab-bar:top {\n"
                                        "    top: 1px;\n"
                                        "}\n"
                                        "\n"
                                        "QTabWidget::tab-bar:bottom {\n"
                                        "    bottom: 1px;\n"
                                        "}\n"
                                        "\n"
                                        "QTabWidget::tab-bar:left {\n"
                                        "    right: 1px;\n"
                                        "}\n"
                                        "\n"
                                        "QTabWidget::tab-bar:right {\n"
                                        "    left: 1px;\n"
                                        "}\n"
                                        "\n"
                                        "\n"
                                        "\n"
                                        "QTabBar::tab {\n"
                                        "    border: 1px solid black;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:selected {\n"
                                        "    background: white;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:!selected {\n"
                                        "    background: silver;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:!selected:hover {\n"
                                        "    background: #999;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:top:!selected {\n"
                                        "    margin-top: 3px;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:bottom:!selected {\n"
                                        "    margin-bottom: 3px;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:top, QTabBar::tab:bottom {\n"
                                        "    min-width: 8ex;\n"
                                        "    margin-right: -1px;\n"
                                        "    padding: 5px 10px 5px 10px;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:top:selected {\n"
                                        "    border-bottom-color: none;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:bottom:selected {\n"
                                        "    border-top-color: none;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:top:last, QTabBar::tab:bottom:last,\n"
                                        "QTabBar::tab:top:only-one, QTabBar::tab:bottom:only-one {\n"
                                        "    margin-right: 0;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:left:!selected {\n"
                                        "    margin-right: 3px;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:right:!selected {\n"
                                        "    margin-left: 3px;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:left, QTabBar::tab:right {\n"
                                        "    min-height: 8ex;\n"
                                        "    margin-bottom: -1px;\n"
                                        "    padding: 10px 5px 10px 5px;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:left:selected {\n"
                                        "    border-left-color: none;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:right:selected {\n"
                                        "    border-right-color: none;\n"
                                        "}\n"
                                        "\n"
                                        "QTabBar::tab:left:last, QTabBar::tab:right:last,\n"
                                        "QTabBar::tab:left:only-one, QTabBar::tab:right:only-one {\n"
                                        "    margin-bottom: 0;\n"
                                        "}\n")
        
        self.ui.tabWidget.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.ui.tabWidget.setDocumentMode(False)
        self.ui.tabWidget.setTabsClosable(False)
        self.ui.tabWidget.setTabBarAutoHide(False)

        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.ui.label_5.setFont(font)

        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.ui.label_10.setFont(font)

        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.ui.label_6.setFont(font)

        font = QtGui.QFont()
        font.setPointSize(9)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.ui.label_13.setFont(font)

        self.ui.time_play_button.setStyleSheet("background: rgb(76, 86, 90);")

        self.ui.time_box.setStyleSheet("background-color: rgb(0, 0, 0);\n"
                                       "color: rgb(236, 236, 236);\n"
                                       "font: 9pt \"MS Shell Dlg 2\";")

        self.ui.annotation_tab.setStyleSheet("font: 9pt;")
        self.ui.settings_tab.setStyleSheet("font: 9pt;")


    def _slider_changed(self):
        # print('Slider changed')
        slider_ratio = float(self.ui.time_slider.value() - self.min_t_slider) / float(self.max_t_slider - self.min_t_slider)
        slider_ratio = saturate(slider_ratio, 0.0, 1.0)

        abs_timestamp = int(slider_ratio * (self.maximum_time - self.minimum_time) + self.minimum_time)
        abs_timestamp = saturate(abs_timestamp, self.minimum_time, self.maximum_time)

        rel_timestamp = saturate(slider_ratio * self.max_rel_time, 0.0, self.max_rel_time)
        
        # No need to trigger ui.time_box changed callbacks
        self.ui.time_box.blockSignals(True)
        self.ui.time_box.setValue(rel_timestamp)
        self.ui.time_box.blockSignals(False)
        self._time_update(abs_timestamp)


    def _time_box_changed(self):
        # print('Time box changed')
        time_box_ratio = self.ui.time_box.value() / self.max_rel_time
        time_box_ratio = saturate(time_box_ratio, 0.0, 1.0)

        abs_timestamp = int(time_box_ratio * (self.maximum_time - self.minimum_time) + self.minimum_time)
        abs_timestamp = saturate(abs_timestamp, self.minimum_time, self.maximum_time)

        slider_value = int(time_box_ratio * (self.max_t_slider - self.min_t_slider) + self.min_t_slider)
        slider_value = saturate(slider_value, self.min_t_slider, self.max_t_slider)

        # No need to trigger ui.time_slider changed callbacks
        self.ui.time_slider.blockSignals(True)
        self.ui.time_slider.setValue(slider_value)
        self.ui.time_slider.blockSignals(False)
        self._time_update(abs_timestamp)

    
    def _time_play_button_timer_callback(self):
        # t0 = time.time()
        if self.ui.time_box.value() >= (self.max_rel_time - 1e-3):
            print("Stopping playback")
            self.time_play_button_timer.stop()
            self.ui.time_play_button.setChecked(False)
            return
        
        time_box_ratio = (self.ui.time_box.value() + float(self.config['play_button_period_ms'] / 1e3)) / self.max_rel_time
        time_box_ratio = saturate(time_box_ratio, 0.0, 1.0)

        abs_timestamp = int(time_box_ratio * (self.maximum_time - self.minimum_time) + self.minimum_time)
        abs_timestamp = saturate(abs_timestamp, self.minimum_time, self.maximum_time)

        slider_value = int(time_box_ratio * (self.max_t_slider - self.min_t_slider) + self.min_t_slider)
        slider_value = saturate(slider_value, self.min_t_slider, self.max_t_slider)

        # No need to trigger ui.time_box and ui.time_slider changed callbacks
        self.ui.time_box.blockSignals(True)
        self.ui.time_slider.blockSignals(True)
        self.ui.time_box.setValue(time_box_ratio * self.max_rel_time)
        self.ui.time_slider.setValue(slider_value)
        self.ui.time_box.blockSignals(False)
        self.ui.time_slider.blockSignals(False)

        self._time_update(abs_timestamp)
        # t1 = time.time()
        # print(f'Play button timer executed, runtime: {(t1-t0) * 1000:.3f}ms')


    def _time_update(self, timestamp: int):
        # print('Time update called')
        # t0 = time.time()
        self.last_cam_rel_time = self.imu_data[self.active_topic].abs2rel_time(timestamp)
        # t1 = time.time()
        self.ui.acc_plot.setXRange(self.last_cam_rel_time - self.config['time_displayed_sec'] / 2, 
                                   self.last_cam_rel_time + self.config['time_displayed_sec'] / 2)

        self.acc_plot_centerline.setValue(self.last_cam_rel_time)
        self.gyr_plot_centerline.setValue(self.last_cam_rel_time)
        self.acc_plot_start_line.setValue(self.last_cam_rel_time - self.ann_start_line_offset)
        self.gyr_plot_start_line.setValue(self.last_cam_rel_time - self.ann_start_line_offset)
        self.acc_plot_end_line.setValue(self.last_cam_rel_time + self.ann_end_line_offset)
        self.gyr_plot_end_line.setValue(self.last_cam_rel_time + self.ann_end_line_offset)
        # print("Setting values for label time boxes...")
        self.ui.t_start_box.setValue(self.acc_plot_start_line.value())
        self.ui.t_end_box.setValue(self.acc_plot_end_line.value())
        # print("FINISHED Setting values for label time boxes...")
        # t2 = time.time()
        # print(f'camera abs2rel_time    : {(t1-t0) * 1000:.3f}ms')
        # print(f'Set vals for UI widgets: {(t2-t1) * 1000:.3f}ms')

        cam_current_idx = binary_search(self.camera_times, timestamp)
        if cam_current_idx != self.cam_last_idx:
            # t0 = time.time()
            qimage = QtGui.QImage.fromData(get_by_path(self.data, ['/image_raw/compressed', 'data', cam_current_idx]))
            # t1 = time.time()
            pixmap = QtGui.QPixmap.fromImage(qimage)
            # t2 = time.time()
            height = int(0.475 * self.ui.main_widget.height())
            width  = int(height * (float)(pixmap.width()) / (float)(pixmap.height()))
            self.ui.camera_view.setPixmap(pixmap.scaled(width, height, QtCore.Qt.IgnoreAspectRatio))
            self.cam_last_idx = cam_current_idx
            # t3 = time.time()
            # print(f'QImage from bytes array: {(t1-t0) * 1000:.3f}ms')
            # print(f'QPixmal from QImage    : {(t2-t1) * 1000:.3f}ms')
            # print(f'setPixmap()            : {(t3-t2) * 1000:.3f}ms')
        # print()


    def _time_play_button_pressed(self):
        # print('Play button clicked-callback called')
        if self.ui.time_play_button.isChecked():
            self.time_play_button_timer.start(int(self.config['play_button_period_ms']))
        else:
            self.time_play_button_timer.stop()


    def _set_imu_topic_on_plots(self, topic: str):
        self.acc_x_plot.setData(self.imu_data[topic].rel_time, self.imu_data[topic].a_x)
        self.acc_y_plot.setData(self.imu_data[topic].rel_time, self.imu_data[topic].a_y)
        self.acc_z_plot.setData(self.imu_data[topic].rel_time, self.imu_data[topic].a_z)
        self.gyr_x_plot.setData(self.imu_data[topic].rel_time, self.imu_data[topic].g_x)
        self.gyr_y_plot.setData(self.imu_data[topic].rel_time, self.imu_data[topic].g_y)
        self.gyr_z_plot.setData(self.imu_data[topic].rel_time, self.imu_data[topic].g_z)
        self.active_topic = topic


    def _filter_settings_changed(self):
        wien_size_value = self.ui.wiener_size_box.value()
        moving_average_value = self.ui.mv_size_box.value()

        for imu_topic in self.imu_data.keys():
            self.imu_data[imu_topic].reset_filters()
            self.imu_data[imu_topic].apply_filter(lambda x: signal_filter.wiener(x, wien_size_value))
            self.imu_data[imu_topic].apply_filter(lambda x: signal_filter.moving_average(x, moving_average_value))

        print(f'Setting wiener filter size to {wien_size_value} and moving average size to {moving_average_value}')
        self._set_imu_topic_on_plots(self.ui.imu_topic_dropdown.currentText())

    
    def _time_displayed_changed(self):
        value = self.ui.time_disp_box.value()
        self.config['time_displayed_sec'] = value
        self.ui.acc_plot.setXRange(self.last_cam_rel_time - value / 2, 
                                   self.last_cam_rel_time + value / 2)
        
    
    def _control_settings_changed(self):
        self.config['previous_time_jump_sec'] = self.ui.prev_t_jump_box.value()
        self.config['next_time_jump_sec'] = self.ui.next_t_jump_box.value()
        

    def _create_annotation_widgets(self, ann: Annotation, idx: int):
        item = QtWidgets.QListWidgetItem()
        item.setText(annotation_text(ann))
        item.setToolTip(annotation_tooltip(ann))
        item.setFlags(item.flags() | QtCore.Qt.ItemFlag.ItemIsUserCheckable)
        item.setCheckState(QtCore.Qt.CheckState.Checked)
        self.ui.annotation_list.insertItem(idx, item)
        self.ui.annotation_list.setMinimumWidth(int(1.075 * self.ui.annotation_list.sizeHintForColumn(0)))

        region_args = {'values': (ann.rel_t_start,
                                  ann.rel_t_end),
                       'movable': False,
                       'orientation': 'vertical',
                       'brush': pg.mkBrush(color=ann.label.anomaly.color),
                       'pen':   pg.mkPen(color=[sum(x) for x in zip(ann.label.anomaly.color, [0,0,0,40])])}
        self.annotation_regions_acc.insert(idx, pg.LinearRegionItem(**region_args))
        self.annotation_regions_gyr.insert(idx, pg.LinearRegionItem(**region_args))
        self.ui.acc_plot.addItem(self.annotation_regions_acc[idx])
        self.ui.gyr_plot.addItem(self.annotation_regions_gyr[idx])


    def _insert_annotation(self):
        rel_time_start = self.ui.t_start_box.value()
        rel_time_end   = self.ui.t_end_box.value()

        if rel_time_start >= rel_time_end:
            warnings.warn('Annotation start-time is greater or equal to end-time')
            return
        
        anomaly_text = self.ui.anomaly_dropdown.currentText()
        transversity_text = self.ui.transversity_dropdown.currentText()
        severity_text = self.ui.severity_dropdown.currentText()
        
        imu_topic = self.ui.imu_topic_dropdown.currentText()
        idx_start = self.imu_data[imu_topic].rel2idx(rel_time_start)
        idx_end   = self.imu_data[imu_topic].rel2idx(rel_time_end)

        ann = Annotation(abs_t_start = self.imu_data[imu_topic].abs_time[idx_start],
                         abs_t_end   = self.imu_data[imu_topic].abs_time[idx_end],
                         rel_t_start = self.imu_data[imu_topic].rel_time[idx_start],
                         rel_t_end   = self.imu_data[imu_topic].rel_time[idx_end],
                         idx_start   = idx_start,
                         idx_end     = idx_end,
                         label       = Label(AnomalyLabel[anomaly_text],
                                             TransversityLabel[transversity_text],
                                             SeverityLabel[severity_text]))
        
        idx = self.annotations.insert(ann)
        self._create_annotation_widgets(ann, idx)


    def _remove_annotation(self):
        current_idx = self.ui.annotation_list.currentRow()
        if(current_idx < 0):
            return
        
        self.annotations.remove(current_idx)
        self.ui.annotation_list.takeItem(current_idx)
        self.ui.annotation_list.setMinimumWidth(int(1.025 * self.ui.annotation_list.sizeHintForColumn(0)))

        self.ui.acc_plot.removeItem(self.annotation_regions_acc[current_idx])
        self.ui.gyr_plot.removeItem(self.annotation_regions_gyr[current_idx])
        self.annotation_regions_acc.pop(current_idx)
        self.annotation_regions_gyr.pop(current_idx)

        # Remove row focus
        self.ui.annotation_list.setCurrentRow(-1)


    def _clicked_annotation(self, item: QtWidgets.QListWidgetItem):
        idx = self.ui.annotation_list.row(item)
        abs_timestamp = (self.annotations.items[idx].abs_t_start + self.annotations.items[idx].abs_t_end) // 2

        time_box_ratio = float(abs_timestamp - self.minimum_time) / float(self.maximum_time - self.minimum_time)
        time_box_ratio = saturate(time_box_ratio, 0.0, 1.0)

        slider_value = int(time_box_ratio * (self.max_t_slider - self.min_t_slider) + self.min_t_slider)
        slider_value = saturate(slider_value, self.min_t_slider, self.max_t_slider)

        # No need to trigger ui.time_box and ui.time_slider changed callbacks
        self.ui.time_box.blockSignals(True)
        self.ui.time_slider.blockSignals(True)
        self.ui.time_box.setValue(time_box_ratio * self.max_rel_time)
        self.ui.time_slider.setValue(slider_value)
        self.ui.time_box.blockSignals(False)
        self.ui.time_slider.blockSignals(False)

        self._time_update(abs_timestamp)

        # Update draggable lines to match annotation time limits
        ann_start = self.annotations.items[idx].rel_t_start
        ann_end   = self.annotations.items[idx].rel_t_end
        self.acc_plot_start_line.setValue(ann_start)
        self.gyr_plot_start_line.setValue(ann_start)
        self.acc_plot_end_line.setValue(ann_end)
        self.gyr_plot_end_line.setValue(ann_end)
        self.ui.t_start_box.blockSignals(True)
        self.ui.t_end_box.blockSignals(True)
        self.ui.t_start_box.setValue(ann_start)
        self.ui.t_end_box.setValue(ann_end)
        self.ui.t_start_box.blockSignals(False)
        self.ui.t_end_box.blockSignals(False)
        self.ann_start_line_offset = self.acc_plot_centerline.value() - self.acc_plot_start_line.value()
        self.ann_end_line_offset   = self.acc_plot_end_line.value()   - self.acc_plot_centerline.value()


    def _changed_annotation_list(self, item: QtWidgets.QListWidgetItem):
        idx = self.ui.annotation_list.row(item)
        if item.checkState() == QtCore.Qt.CheckState.Checked:
            print(f"Turning visible annotation with index = {idx}")
            self.annotation_regions_acc[idx].setVisible(True)
            self.annotation_regions_gyr[idx].setVisible(True)
        elif item.checkState() == QtCore.Qt.CheckState.Unchecked:
            print(f"Turning invisible annotation with index = {idx}")
            self.annotation_regions_acc[idx].setVisible(False)
            self.annotation_regions_gyr[idx].setVisible(False)
        

    def _dump_annotations(self):
        name, _ = QtWidgets.QFileDialog.getSaveFileName(self.main_window, "Dump file", options=QtWidgets.QFileDialog.DontUseNativeDialog,
                                                        directory=os.path.abspath(os.path.dirname(self.pickle_filepath)))
        if name:
            file_entries = {'file': os.path.basename(self.pickle_filepath),
                            'date': time.strftime('%A %d-%m-%Y %H:%M:%S', time.localtime(self.minimum_time // 1e9))}
            self.annotations.to_json(name, file_entries)


    def _load_annotations(self):
        name, _ = QtWidgets.QFileDialog.getOpenFileName(self.main_window, "Load file", options=QtWidgets.QFileDialog.DontUseNativeDialog, 
                                                        directory=os.path.abspath(os.path.dirname(self.pickle_filepath)))
        if name:
            self.annotations.from_json(name)

            self.ui.annotation_list.clear()
            for idx, ann in enumerate(self.annotations.items):
                self._create_annotation_widgets(ann, idx)


    def _ann_start_dragged(self, obj):
        if obj is self.acc_plot_start_line:
            self.ann_start_line_offset = self.acc_plot_centerline.value() - self.acc_plot_start_line.value()
            self.gyr_plot_start_line.setValue(self.acc_plot_start_line.value())
            # print(f'Dragged accelerometer annotation start line')
        elif obj is self.gyr_plot_start_line:
            self.ann_start_line_offset = self.gyr_plot_centerline.value() - self.gyr_plot_start_line.value()
            self.acc_plot_start_line.setValue(self.gyr_plot_start_line.value())
            # print(f'Dragged gyroscope annotation start line')


    def _ann_end_dragged(self, obj):
        if obj is self.acc_plot_end_line:
            self.ann_end_line_offset = self.acc_plot_end_line.value() - self.acc_plot_centerline.value()
            self.gyr_plot_end_line.setValue(self.acc_plot_end_line.value())
            # print(f'Dragged accelerometer annotation end line')
        elif obj is self.gyr_plot_end_line:
            self.ann_end_line_offset = self.gyr_plot_end_line.value() - self.gyr_plot_centerline.value()
            self.acc_plot_end_line.setValue(self.gyr_plot_end_line.value())
            # print(f'Dragged gyroscope annotation end line')


    def _ann_start_pos_changed(self, obj):
        if obj is self.acc_plot_start_line:
            self.ui.t_start_box.setValue(self.acc_plot_start_line.value())
            # print(f'Finished changing pos to accelerometer annotation start line')
        elif obj is self.gyr_plot_start_line:
            self.ui.t_start_box.setValue(self.gyr_plot_start_line.value())
            # print(f'Finished changing pos to gyroscope annotation start line')

    
    def _ann_end_pos_changed(self, obj):
        if obj is self.acc_plot_end_line:
            self.ui.t_end_box.setValue(self.acc_plot_end_line.value())
            # print(f'Finished changing pos to accelerometer annotation end line')
        elif obj is self.gyr_plot_end_line:
            self.ui.t_end_box.setValue(self.gyr_plot_end_line.value())
            # print(f'Finished changing pos to gyroscope annotation end line')


    def _ann_time_start_box_changed(self):
        # t0 = time.time()
        self.acc_plot_start_line.setValue(self.ui.t_start_box.value())
        self.gyr_plot_start_line.setValue(self.ui.t_start_box.value())
        self.ann_start_line_offset = self.acc_plot_centerline.value() - self.acc_plot_start_line.value()
        # t1 = time.time()
        # print(f'Changed t_start_box value to: {self.ui.t_start_box.value()}, time needed: {(t1-t0) * 1000:.3f}ms')


    def _ann_time_end_box_changed(self):
        # t0 = time.time()
        self.acc_plot_end_line.setValue(self.ui.t_end_box.value())
        self.gyr_plot_end_line.setValue(self.ui.t_end_box.value())
        self.ann_end_line_offset = self.acc_plot_end_line.value() - self.acc_plot_centerline.value()
        # t1 = time.time()
        # print(f'Changed t_end_box value to: {self.ui.t_end_box.value()}, time needed: {(t1-t0) * 1000:.3f}ms')
