import os
os.add_dll_directory("C:\\bin\\gstreamer\\1.0\\msvc_x86_64\\bin")
import sys
import cv2
import numpy as np
import json
import os
import subprocess
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QCheckBox,
                             QTabWidget, QGridLayout, QProgressBar, QTextEdit)
from PyQt5.QtGui import QImage, QPixmap, QIcon, QPainter, QColor
from PyQt5.QtCore import Qt, QTimer, QSize, QEvent
from pymavlink import mavutil
import threading
import time


class VideoTelemetryWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Video and Telemetry with Enhanced OSD")
        self.setGeometry(100, 100, 1000, 600)

        # GStreamer pipeline
        self.pipeline_str = (
            "udpsrc port=5700 ! "
            "application/x-rtp,encoding-name=H264,payload=96,clock-rate=90000,media=video ! "
            "rtpjitterbuffer latency=100 ! rtph264depay ! h264parse ! "
            "openh264dec ! videoconvert ! "
            "appsink emit-signals=True drop=True max-buffers=1"
        )

        # Video capture
        self.cap = cv2.VideoCapture(self.pipeline_str, cv2.CAP_GSTREAMER)
        self.video_writer_osd = None
        self.video_writer_raw = None

        # MAVLink connection
        self.mavlink_conn = None

        # Video parameters
        self.frame_width = 640
        self.frame_height = 480

        # Recording state
        self.is_recording = False
        self.recording_path = ""

        # OSD elements configuration
        self.osd_elements = {
            'roll': {'pos': (10, 30), 'enabled': True},
            'pitch': {'pos': (10, 60), 'enabled': True},
            'yaw': {'pos': (10, 90), 'enabled': True},
            'speed': {'pos': (self.frame_width - 150, 30), 'enabled': True},
            'altitude': {'pos': (self.frame_width - 150, 60), 'enabled': True},
            'battery': {'pos': (self.frame_width - 150, 90), 'enabled': True},
            'horizon': {'pos': (self.frame_width // 2, self.frame_height // 2), 'enabled': True},
            'crosshair': {'pos': (self.frame_width // 2, self.frame_height // 2), 'enabled': True},
            'vibration': {'pos': (self.frame_width - 150, 120), 'enabled': True},
            'compass': {'pos': (self.frame_width // 2, self.frame_height - 50), 'enabled': True},
            'ekf': {'pos': (10, 120), 'enabled': True}
        }
        self.selected_element = None
        self.drag_offset = None

        # Additional telemetry data
        self.vibration_data = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'clip': 0}
        self.ekf_status = {
            'flags': 0,
            'velocity': 0.0,
            'pos_horiz': 0.0,
            'pos_vert': 0.0,
            'compass': 0.0
        }

        # Initialize osd_checkboxes
        self.osd_checkboxes = {}

        # GUI setup
        self.setup_ui()

        # MAVLink connection
        self.connect_mavlink()

        # Load OSD settings after UI setup
        self.load_osd_settings()

        # Telemetry data
        self.telemetry_data = {
            'attitude': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'velocity': {'speed': 0.0, 'heading': 0},
            'position': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0},
            'battery': {'remaining': 0}
        }

        # Flags
        self.running = True

        # Start threads
        self.video_thread = threading.Thread(target=self.update_video)
        self.video_thread.daemon = True
        self.video_thread.start()

        self.telemetry_thread = threading.Thread(target=self.update_telemetry)
        self.telemetry_thread.daemon = True
        self.telemetry_thread.start()

        # Timer for GUI update
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(33)

    def setup_ui(self):
        main_widget = QWidget(self)
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # Video display
        self.video_label = QLabel()
        self.video_label.setMinimumSize(self.frame_width, self.frame_height)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.installEventFilter(self)

        # Right panel with tabs
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)

        # Tab widget
        tabs = QTabWidget()

        # Telemetry tab
        telemetry_tab = QWidget()
        telemetry_layout = QVBoxLayout(telemetry_tab)
        self.attitude_label = QLabel("Attitude: Roll: 0.0, Pitch: 0.0, Yaw: 0.0")
        self.velocity_label = QLabel("Velocity: Speed: 0.0 m/s, Hd: 0")
        self.position_label = QLabel("Position: Lat: 0.0, Lon: 0.0, Alt: 0.0m")
        self.battery_label = QLabel("Battery: 0%")

        # Vibration and EKF bars in one row
        status_layout = QHBoxLayout()
        self.vibe_x_bar = QProgressBar()
        self.vibe_x_bar.setMaximum(100)
        self.vibe_x_bar.setOrientation(Qt.Vertical)
        self.vibe_y_bar = QProgressBar()
        self.vibe_y_bar.setMaximum(100)
        self.vibe_y_bar.setOrientation(Qt.Vertical)
        self.vibe_z_bar = QProgressBar()
        self.vibe_z_bar.setMaximum(100)
        self.vibe_z_bar.setOrientation(Qt.Vertical)
        self.ekf_vel_bar = QProgressBar()
        self.ekf_vel_bar.setMaximum(100)
        self.ekf_vel_bar.setOrientation(Qt.Vertical)
        self.ekf_pos_h_bar = QProgressBar()
        self.ekf_pos_h_bar.setMaximum(100)
        self.ekf_pos_h_bar.setOrientation(Qt.Vertical)
        self.ekf_pos_v_bar = QProgressBar()
        self.ekf_pos_v_bar.setMaximum(100)
        self.ekf_pos_v_bar.setOrientation(Qt.Vertical)
        self.ekf_comp_bar = QProgressBar()
        self.ekf_comp_bar.setMaximum(100)
        self.ekf_comp_bar.setOrientation(Qt.Vertical)

        status_layout.addWidget(QLabel("Vibe X"))
        status_layout.addWidget(self.vibe_x_bar)
        status_layout.addWidget(QLabel("Y"))
        status_layout.addWidget(self.vibe_y_bar)
        status_layout.addWidget(QLabel("Z"))
        status_layout.addWidget(self.vibe_z_bar)
        status_layout.addWidget(QLabel("EKF Vel"))
        status_layout.addWidget(self.ekf_vel_bar)
        status_layout.addWidget(QLabel("PosH"))
        status_layout.addWidget(self.ekf_pos_h_bar)
        status_layout.addWidget(QLabel("PosV"))
        status_layout.addWidget(self.ekf_pos_v_bar)
        status_layout.addWidget(QLabel("Comp"))
        status_layout.addWidget(self.ekf_comp_bar)

        self.vibe_label = QLabel("Vibration: X: 0.0, Y: 0.0, Z: 0.0, Clip: 0")
        self.ekf_label = QLabel("EKF Status: OK")

        # Recording layout with label and button
        record_layout = QHBoxLayout()
        self.record_label = QLabel("Recording: Not active")
        self.open_folder_button = QPushButton("Open Folder")
        self.open_folder_button.clicked.connect(self.open_recording_folder)
        self.open_folder_button.setEnabled(False)
        record_layout.addWidget(self.record_label)
        record_layout.addWidget(self.open_folder_button)

        self.record_button = QPushButton(" Record")
        self.record_button.setMinimumWidth(100)
        self.update_record_button_icon()
        self.record_button.clicked.connect(self.toggle_recording)

        # Status bar (textarea)
        self.status_bar = QTextEdit()
        self.status_bar.setReadOnly(True)
        self.status_bar.setMaximumHeight(100)
        self.status_bar.setText("Application started")

        telemetry_layout.addWidget(self.attitude_label)
        telemetry_layout.addWidget(self.velocity_label)
        telemetry_layout.addWidget(self.position_label)
        telemetry_layout.addWidget(self.battery_label)
        telemetry_layout.addLayout(status_layout)
        telemetry_layout.addWidget(self.vibe_label)
        telemetry_layout.addWidget(self.ekf_label)
        telemetry_layout.addLayout(record_layout)
        telemetry_layout.addWidget(self.record_button)
        telemetry_layout.addWidget(self.status_bar)
        telemetry_layout.addStretch()

        # OSD settings tab
        osd_tab = QWidget()
        osd_layout = QGridLayout(osd_tab)
        for i, element in enumerate(self.osd_elements):
            cb = QCheckBox(f"Show {element.capitalize()}")
            cb.setChecked(self.osd_elements[element]['enabled'])
            cb.stateChanged.connect(lambda state, e=element: self.toggle_osd_element(e, state))
            self.osd_checkboxes[element] = cb
            osd_layout.addWidget(cb, i, 0)

        save_button = QPushButton("Save Settings")
        save_button.clicked.connect(self.save_osd_settings)
        load_button = QPushButton("Load Settings")
        load_button.clicked.connect(self.load_osd_settings)
        osd_layout.addWidget(save_button, len(self.osd_elements), 0)
        osd_layout.addWidget(load_button, len(self.osd_elements), 1)

        tabs.addTab(telemetry_tab, "Telemetry")
        tabs.addTab(osd_tab, "OSD Settings")

        right_layout.addWidget(tabs)
        main_layout.addWidget(self.video_label)
        main_layout.addWidget(right_widget)

    def toggle_osd_element(self, element, state):
        self.osd_elements[element]['enabled'] = (state == Qt.Checked)

    def save_osd_settings(self):
        settings = {key: {'pos': list(val['pos']), 'enabled': val['enabled']}
                    for key, val in self.osd_elements.items()}
        with open('osd_settings.json', 'w') as f:
            json.dump(settings, f)
        self.status_bar.append("OSD settings saved")
        print("OSD settings saved")

    def load_osd_settings(self):
        if os.path.exists('osd_settings.json'):
            with open('osd_settings.json', 'r') as f:
                settings = json.load(f)
                for key in self.osd_elements:
                    if key in settings:
                        self.osd_elements[key]['pos'] = tuple(settings[key]['pos'])
                        self.osd_elements[key]['enabled'] = settings[key]['enabled']
                        if hasattr(self, 'osd_checkboxes') and key in self.osd_checkboxes:
                            self.osd_checkboxes[key].setChecked(settings[key]['enabled'])
            self.status_bar.append("OSD settings loaded")
            print("OSD settings loaded")
        else:
            self.status_bar.append("No OSD settings file found")
            print("No OSD settings file found")

    def update_record_button_icon(self):
        pixmap = QPixmap(20, 20)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        if self.is_recording:
            painter.setBrush(QColor(255, 0, 0))
            painter.drawEllipse(2, 2, 16, 16)
        else:
            painter.setBrush(QColor(128, 128, 128))
            painter.drawRect(4, 4, 12, 12)
        painter.end()
        self.record_button.setIcon(QIcon(pixmap))
        self.record_button.setIconSize(QSize(20, 20))

    def connect_mavlink(self):
        try:
            self.mavlink_conn = mavutil.mavlink_connection('udp:0.0.0.0:14550')
            self.status_bar.append("Connected to MAVLink UDP port 14550")
            print("Connected to MAVLink UDP port 14550")
        except Exception as e:
            self.status_bar.append(f"Failed to connect to MAVLink: {e}")
            print(f"Failed to connect to MAVLink: {e}")
            self.mavlink_conn = None

    def toggle_recording(self):
        if not self.is_recording:
            self.is_recording = True
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            self.recording_path = f'recording_{timestamp}'
            osd_file = f'{self.recording_path}_osd.avi'  # Changed to .avi
            raw_file = f'{self.recording_path}_raw.avi'  # Changed to .avi

            # Try H.264 first, fall back to XVID if it fails
            fourcc = cv2.VideoWriter_fourcc(*'H264')
            self.video_writer_osd = cv2.VideoWriter(
                osd_file, fourcc, 30.0, (self.frame_width, self.frame_height), isColor=True
            )
            self.video_writer_raw = cv2.VideoWriter(
                raw_file, fourcc, 30.0, (self.frame_width, self.frame_height), isColor=True
            )

            # Check if H.264 failed
            if not self.video_writer_osd.isOpened() or not self.video_writer_raw.isOpened():
                self.status_bar.append("H.264 codec failed, falling back to XVID")
                print("H.264 codec failed, falling back to XVID")
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_writer_osd = cv2.VideoWriter(
                    osd_file, fourcc, 30.0, (self.frame_width, self.frame_height), isColor=True
                )
                self.video_writer_raw = cv2.VideoWriter(
                    raw_file, fourcc, 30.0, (self.frame_width, self.frame_height), isColor=True
                )

            if self.video_writer_osd.isOpened() and self.video_writer_raw.isOpened():
                self.record_label.setText("Recording: Active")
                self.status_bar.append(f"Recording started:\n{osd_file}\n{raw_file}")
                self.open_folder_button.setEnabled(True)
                print("Recording started")
            else:
                self.status_bar.append("Failed to initialize video writers")
                print("Failed to initialize video writers")
                self.is_recording = False
                self.open_folder_button.setEnabled(False)
        else:
            self.is_recording = False
            if self.video_writer_osd:
                self.video_writer_osd.release()
            if self.video_writer_raw:
                self.video_writer_raw.release()
            self.record_label.setText("Recording: Not active")
            self.status_bar.append("Recording stopped")
            self.open_folder_button.setEnabled(False)
            print("Recording stopped")
        self.update_record_button_icon()

    def open_recording_folder(self):
        folder_path = os.path.dirname(os.path.abspath(f"{self.recording_path}_osd.avi"))
        if os.path.exists(folder_path):
            if sys.platform == "win32":
                os.startfile(folder_path)
            elif sys.platform == "darwin":  # macOS
                subprocess.call(["open", folder_path])
            else:  # Linux
                subprocess.call(["xdg-open", folder_path])
            self.status_bar.append(f"Opened folder: {folder_path}")
        else:
            self.status_bar.append(f"Folder not found: {folder_path}")
            print(f"Folder not found: {folder_path}")

    def draw_osd(self, frame):
        osd_frame = frame.copy()
        color = (255, 255, 255)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1

        if self.osd_elements['horizon']['enabled']:
            center_x, center_y = self.osd_elements['horizon']['pos']
            roll = self.telemetry_data['attitude']['roll']
            pitch = self.telemetry_data['attitude']['pitch']
            line_length = 100
            angle = -roll
            pitch_offset = int(pitch * 10)
            x1 = int(center_x - line_length * np.cos(angle))
            y1 = int(center_y - pitch_offset + line_length * np.sin(angle))
            x2 = int(center_x + line_length * np.cos(angle))
            y2 = int(center_y - pitch_offset - line_length * np.sin(angle))
            cv2.line(osd_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if self.osd_elements['crosshair']['enabled']:
            center_x, center_y = self.osd_elements['crosshair']['pos']
            cv2.line(osd_frame, (center_x - 10, center_y), (center_x + 10, center_y), color, 1)
            cv2.line(osd_frame, (center_x, center_y - 10), (center_x, center_y + 10), color, 1)

        if self.osd_elements['roll']['enabled']:
            cv2.putText(osd_frame, f"Roll: {self.telemetry_data['attitude']['roll']:.1f}",
                        self.osd_elements['roll']['pos'], font, font_scale, color, thickness)
        if self.osd_elements['pitch']['enabled']:
            cv2.putText(osd_frame, f"Pitch: {self.telemetry_data['attitude']['pitch']:.1f}",
                        self.osd_elements['pitch']['pos'], font, font_scale, color, thickness)
        if self.osd_elements['yaw']['enabled']:
            cv2.putText(osd_frame, f"Yaw: {self.telemetry_data['attitude']['yaw']:.1f}",
                        self.osd_elements['yaw']['pos'], font, font_scale, color, thickness)
        if self.osd_elements['speed']['enabled']:
            cv2.putText(osd_frame, f"Speed: {self.telemetry_data['velocity']['speed']:.1f} m/s",
                        self.osd_elements['speed']['pos'], font, font_scale, color, thickness)
        if self.osd_elements['altitude']['enabled']:
            cv2.putText(osd_frame, f"Alt: {self.telemetry_data['position']['alt']:.1f} m",
                        self.osd_elements['altitude']['pos'], font, font_scale, color, thickness)
        if self.osd_elements['battery']['enabled']:
            cv2.putText(osd_frame, f"Bat: {self.telemetry_data['battery']['remaining']}%",
                        self.osd_elements['battery']['pos'], font, font_scale, color, thickness)

        if self.osd_elements['vibration']['enabled']:
            x, y = self.osd_elements['vibration']['pos']
            bar_height = 50
            cv2.rectangle(osd_frame, (x, y), (x + 10, y - bar_height), (0, 0, 0), 1)
            cv2.rectangle(osd_frame, (x, y - int(abs(self.vibration_data['x']) / 100 * bar_height)), (x + 10, y),
                          (0, 255, 0) if abs(self.vibration_data['x']) < 30 else (0, 0, 255), -1)
            cv2.rectangle(osd_frame, (x + 15, y), (x + 25, y - bar_height), (0, 0, 0), 1)
            cv2.rectangle(osd_frame, (x + 15, y - int(abs(self.vibration_data['y']) / 100 * bar_height)), (x + 25, y),
                          (0, 255, 0) if abs(self.vibration_data['y']) < 30 else (0, 0, 255), -1)
            cv2.rectangle(osd_frame, (x + 30, y), (x + 40, y - bar_height), (0, 0, 0), 1)
            cv2.rectangle(osd_frame, (x + 30, y - int(abs(self.vibration_data['z']) / 100 * bar_height)), (x + 40, y),
                          (0, 255, 0) if abs(self.vibration_data['z']) < 30 else (0, 0, 255), -1)
            cv2.putText(osd_frame, "V", (x + 15, y + 15), font, font_scale, color, thickness)

        if self.osd_elements['compass']['enabled']:
            center_x, center_y = self.osd_elements['compass']['pos']
            radius = 30
            yaw = self.telemetry_data['attitude']['yaw']
            cv2.circle(osd_frame, (center_x, center_y), radius, color, 1)
            needle_x = int(center_x + radius * np.sin(yaw))
            needle_y = int(center_y - radius * np.cos(yaw))
            cv2.line(osd_frame, (center_x, center_y), (needle_x, needle_y), (0, 0, 255), 2)
            for angle, label in [(0, 'N'), (np.pi / 2, 'E'), (np.pi, 'S'), (3 * np.pi / 2, 'W')]:
                x = int(center_x + (radius + 10) * np.sin(angle))
                y = int(center_y - (radius + 10) * np.cos(angle))
                cv2.putText(osd_frame, label, (x - 5, y + 5), font, font_scale, color, thickness)

        if self.osd_elements['ekf']['enabled']:
            x, y = self.osd_elements['ekf']['pos']
            bar_height = 50
            cv2.rectangle(osd_frame, (x, y), (x + 10, y - bar_height), (0, 0, 0), 1)
            cv2.rectangle(osd_frame, (x, y - int(self.ekf_status['velocity'] * 100)), (x + 10, y),
                          (0, 255, 0) if self.ekf_status['velocity'] < 0.5 else (0, 0, 255), -1)
            cv2.rectangle(osd_frame, (x + 15, y), (x + 25, y - bar_height), (0, 0, 0), 1)
            cv2.rectangle(osd_frame, (x + 15, y - int(self.ekf_status['pos_horiz'] * 100)), (x + 25, y),
                          (0, 255, 0) if self.ekf_status['pos_horiz'] < 0.5 else (0, 0, 255), -1)
            cv2.rectangle(osd_frame, (x + 30, y), (x + 40, y - bar_height), (0, 0, 0), 1)
            cv2.rectangle(osd_frame, (x + 30, y - int(self.ekf_status['pos_vert'] * 100)), (x + 40, y),
                          (0, 255, 0) if self.ekf_status['pos_vert'] < 0.5 else (0, 0, 255), -1)
            cv2.rectangle(osd_frame, (x + 45, y), (x + 55, y - bar_height), (0, 0, 0), 1)
            cv2.rectangle(osd_frame, (x + 45, y - int(self.ekf_status['compass'] * 100)), (x + 55, y),
                          (0, 255, 0) if self.ekf_status['compass'] < 0.5 else (0, 0, 255), -1)
            cv2.putText(osd_frame, "EKF", (x + 15, y + 15), font, font_scale, color, thickness)

        return osd_frame

    def eventFilter(self, obj, event):
        if obj == self.video_label:
            if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
                pos = event.pos()
                for element, config in self.osd_elements.items():
                    x, y = config['pos']
                    if abs(x - pos.x()) < 50 and abs(y - pos.y()) < 20 and config['enabled']:
                        self.selected_element = element
                        self.drag_offset = (pos.x() - x, pos.y() - y)
                        break
            elif event.type() == QEvent.MouseMove and self.selected_element:
                pos = event.pos()
                new_x = pos.x() - self.drag_offset[0]
                new_y = pos.y() - self.drag_offset[1]
                new_x = max(0, min(new_x, self.frame_width - 50))
                new_y = max(0, min(new_y, self.frame_height - 20))
                self.osd_elements[self.selected_element]['pos'] = (new_x, new_y)
            elif event.type() == QEvent.MouseButtonRelease and self.selected_element:
                self.selected_element = None
                self.drag_offset = None
        return super().eventFilter(obj, event)

    def update_video(self):
        while self.running:
            if not self.cap.isOpened():
                self.status_bar.append("Video capture not opened")
                print("Video capture not opened")
                time.sleep(1)
                continue

            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, (self.frame_width, self.frame_height))
                self.current_frame_raw = frame
                self.current_frame_osd = self.draw_osd(frame)
                if self.is_recording:
                    if self.video_writer_osd:
                        self.video_writer_osd.write(self.current_frame_osd)
                    if self.video_writer_raw:
                        self.video_writer_raw.write(self.current_frame_raw)
            else:
                self.status_bar.append("Failed to grab frame")
                print("Failed to grab frame")
                self.current_frame_raw = np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8)
                self.current_frame_osd = self.current_frame_raw.copy()
                time.sleep(0.1)

    def update_telemetry(self):
        while self.running:
            if self.mavlink_conn is None:
                time.sleep(1)
                self.connect_mavlink()
                continue

            try:
                msg = self.mavlink_conn.recv_match(blocking=True, timeout=1.0)

                if msg is not None:
                    msg_type = msg.get_type()

                    if msg_type == 'ATTITUDE':
                        self.telemetry_data['attitude']['roll'] = msg.roll
                        self.telemetry_data['attitude']['pitch'] = msg.pitch
                        self.telemetry_data['attitude']['yaw'] = msg.yaw
                    elif msg_type == 'VFR_HUD':
                        self.telemetry_data['velocity']['speed'] = msg.groundspeed
                        self.telemetry_data['velocity']['heading'] = msg.heading
                    elif msg_type == 'GLOBAL_POSITION_INT':
                        self.telemetry_data['position']['lat'] = msg.lat / 1e7
                        self.telemetry_data['position']['lon'] = msg.lon / 1e7
                        self.telemetry_data['position']['alt'] = msg.relative_alt / 1000
                    elif msg_type == 'BATTERY_STATUS':
                        self.telemetry_data['battery']['remaining'] = msg.battery_remaining
                    elif msg_type == 'VIBRATION':
                        self.vibration_data['x'] = msg.vibration_x
                        self.vibration_data['y'] = msg.vibration_y
                        self.vibration_data['z'] = msg.vibration_z
                        self.vibration_data['clip'] = msg.clipping_0
                    elif msg_type == 'EKF_STATUS_REPORT':
                        self.ekf_status['flags'] = msg.flags
                        self.ekf_status['velocity'] = min(msg.velocity_variance, 1.0)
                        self.ekf_status['pos_horiz'] = min(msg.pos_horiz_variance, 1.0)
                        self.ekf_status['pos_vert'] = min(msg.pos_vert_variance, 1.0)
                        self.ekf_status['compass'] = min(msg.compass_variance, 1.0)

            except Exception as e:
                self.status_bar.append(f"Telemetry error: {e}")
                print(f"Telemetry error: {e}")
                time.sleep(0.1)

    def update_frame(self):
        if hasattr(self, 'current_frame_osd'):
            frame = cv2.cvtColor(self.current_frame_osd, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.video_label.setPixmap(pixmap)

            self.attitude_label.setText(
                f"Attitude: Roll: {self.telemetry_data['attitude']['roll']:.2f}, "
                f"Pitch: {self.telemetry_data['attitude']['pitch']:.2f}, "
                f"Yaw: {self.telemetry_data['attitude']['yaw']:.2f}"
            )
            self.velocity_label.setText(
                f"Velocity: Speed: {self.telemetry_data['velocity']['speed']:.2f} m/s, "
                f"Hd: {self.telemetry_data['velocity']['heading']}"
            )
            self.position_label.setText(
                f"Position: Lat: {self.telemetry_data['position']['lat']:.6f}, "
                f"Lon: {self.telemetry_data['position']['lon']:.6f}, "
                f"Alt: {self.telemetry_data['position']['alt']:.2f}m"
            )
            self.battery_label.setText(
                f"Battery: {self.telemetry_data['battery']['remaining']}%"
            )
            self.vibe_x_bar.setValue(min(int(abs(self.vibration_data['x'])), 100))
            self.vibe_y_bar.setValue(min(int(abs(self.vibration_data['y'])), 100))
            self.vibe_z_bar.setValue(min(int(abs(self.vibration_data['z'])), 100))
            self.vibe_label.setText(
                f"Vibration: X: {self.vibration_data['x']:.1f}, "
                f"Y: {self.vibration_data['y']:.1f}, "
                f"Z: {self.vibration_data['z']:.1f}, Clip: {self.vibration_data['clip']}"
            )
            self.ekf_vel_bar.setValue(int(self.ekf_status['velocity'] * 100))
            self.ekf_pos_h_bar.setValue(int(self.ekf_status['pos_horiz'] * 100))
            self.ekf_pos_v_bar.setValue(int(self.ekf_status['pos_vert'] * 100))
            self.ekf_comp_bar.setValue(int(self.ekf_status['compass'] * 100))
            ekf_text = "EKF Status: OK" if self.ekf_status[
                                               'flags'] == 0 else f"EKF Status: Flags {self.ekf_status['flags']}"
            self.ekf_label.setText(ekf_text)

    def closeEvent(self, event):
        self.running = False
        self.cap.release()
        if self.video_writer_osd:
            self.video_writer_osd.release()
        if self.video_writer_raw:
            self.video_writer_raw.release()
        if self.mavlink_conn:
            self.mavlink_conn.close()
        self.save_osd_settings()
        self.status_bar.append("Application closed")
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = VideoTelemetryWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()