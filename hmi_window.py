"""
Main HMI Window for Nextup Cobot HMI - Compact Version
"""
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QSplitter, QLabel, QPushButton, QSlider,
    QStatusBar, QMessageBox, QGroupBox, QCheckBox,
    QMenu, QDialog, QDialogButtonBox, QLineEdit, QComboBox,
    QFormLayout, QListWidget, QListWidgetItem,
    QFrame, QScrollArea, QButtonGroup, QRadioButton,
    QTreeWidget, QTreeWidgetItem, QHeaderView,
    QSizePolicy
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QSettings, QThread
from PyQt6.QtGui import QAction, QKeySequence, QPalette, QColor, QFont, QCursor
from joint_control_widget import JointControlWidget
from cartesian_control_widget import CartesianControlWidget
from viewer_widget import Robot3DViewer
from servo_controller import ServoController
from theme import Palette, app_qss, button_qss, section_title_qss, checkbox_qss, slider_qss, mono_font

# Try to import MoveIt planner
try:
    from moveit_planner import MoveItPlanner
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    print("⚠ MoveIt planner not available")

from ros_worker import RosWorker
from sensor_msgs.msg import JointState

# ROS2 introspection imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    import subprocess
    import re
    ROS2_INTROSPECTION_AVAILABLE = True
except ImportError:
    ROS2_INTROSPECTION_AVAILABLE = False
    print("⚠ ROS2 introspection not available")


class RosStatusWorker(QThread):
    """Background thread to collect ROS system status"""
    status_updated = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.running = True
        self.node = None
        
    def run(self):
        while self.running:
            try:
                status = self.get_ros_status()
                self.status_updated.emit(status)
                self.msleep(2000)
            except Exception:
                pass
    
    def get_ros_status(self):
        """Collect ROS2 system status"""
        status = {
            'nodes': [],
            'topics': [],
            'services': [],
            'node_count': 0,
            'topic_count': 0,
            'service_count': 0,
            'active': False
        }
        
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                nodes = [n.strip() for n in result.stdout.split('\n') if n.strip()]
                status['nodes'] = nodes
                status['node_count'] = len(nodes)
                status['active'] = True
            
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                topics = [t.strip() for t in result.stdout.split('\n') if t.strip()]
                status['topics'] = topics
                status['topic_count'] = len(topics)
            
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                services = [s.strip() for s in result.stdout.split('\n') if s.strip()]
                status['services'] = services
                status['service_count'] = len(services)
                
        except Exception as e:
            print(f"Error getting ROS status: {e}")
            
        return status
    
    def stop(self):
        self.running = False


class RosStatusPanel(QWidget):
    """Panel showing ROS2 system status"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None
        self.init_ui()
        self.start_monitoring()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(10)
        
        # Title - smaller
        title = QLabel("🤖 ROS2 STATUS")
        title.setStyleSheet("""
            font-size: 16px;
            font-weight: bold;
            color: #22d3ee;
            padding: 5px 0;
            border-bottom: 1px solid #232d3d;
        """)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)
        
        # Status - compact
        status_widget = QWidget()
        status_layout = QHBoxLayout(status_widget)
        status_layout.setContentsMargins(0, 5, 0, 5)
        
        self.status_dot = QLabel("●")
        self.status_dot.setStyleSheet("color: #ffb020; font-size: 16px;")
        status_layout.addWidget(self.status_dot)
        
        self.status_text = QLabel("Monitoring...")
        self.status_text.setStyleSheet("color: #dbe4f0; font-size: 12px;")
        status_layout.addWidget(self.status_text)
        status_layout.addStretch()
        layout.addWidget(status_widget)
        
        # Stats - compact grid
        stats_widget = QWidget()
        stats_layout = QHBoxLayout(stats_widget)
        stats_layout.setSpacing(10)
        
        self.node_count_label = self._create_stat_label("📦", "0")
        stats_layout.addWidget(self.node_count_label)
        
        self.topic_count_label = self._create_stat_label("📡", "0")
        stats_layout.addWidget(self.topic_count_label)
        
        self.service_count_label = self._create_stat_label("🔧", "0")
        stats_layout.addWidget(self.service_count_label)
        
        layout.addWidget(stats_widget)
        
        # Node list - compact
        node_label = QLabel("Nodes:")
        node_label.setStyleSheet("color: #67e8f9; font-weight: bold; font-size: 11px; margin-top: 5px;")
        layout.addWidget(node_label)
        
        self.node_tree = QTreeWidget()
        self.node_tree.setHeaderLabels(["Node", "Status"])
        self.node_tree.setStyleSheet("""
            QTreeWidget {
                background-color: #0d121b;
                border: 1px solid #232d3d;
                border-radius: 4px;
                color: #dbe4f0;
                font-family: monospace;
                font-size: 10px;
                max-height: 120px;
            }
            QTreeWidget::item {
                padding: 2px 4px;
            }
            QTreeWidget::item:selected {
                background-color: #232d3d;
            }
            QHeaderView::section {
                background-color: #111826;
                color: #67e8f9;
                padding: 2px 4px;
                border: none;
                border-bottom: 1px solid #232d3d;
                font-size: 9px;
            }
        """)
        self.node_tree.setMaximumHeight(120)
        self.node_tree.header().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        layout.addWidget(self.node_tree)
        
        # Topic list - compact
        topic_label = QLabel("Topics:")
        topic_label.setStyleSheet("color: #67e8f9; font-weight: bold; font-size: 11px; margin-top: 5px;")
        layout.addWidget(topic_label)
        
        self.topic_list = QListWidget()
        self.topic_list.setStyleSheet("""
            QListWidget {
                background-color: #0d121b;
                border: 1px solid #232d3d;
                border-radius: 4px;
                color: #8492a6;
                font-family: monospace;
                font-size: 9px;
                max-height: 80px;
            }
            QListWidget::item {
                padding: 2px 4px;
            }
            QListWidget::item:selected {
                background-color: #232d3d;
            }
        """)
        self.topic_list.setMaximumHeight(80)
        layout.addWidget(self.topic_list)
        
        # Refresh button - compact
        refresh_btn = QPushButton("🔄 Refresh")
        refresh_btn.setStyleSheet("""
            QPushButton {
                background-color: #123240;
                color: #7dd3e0;
                border: 1px solid #1c4a57;
                border-radius: 4px;
                padding: 4px 8px;
                font-size: 10px;
                margin-top: 5px;
            }
            QPushButton:hover {
                background-color: #14313d;
            }
        """)
        refresh_btn.clicked.connect(self.force_refresh)
        layout.addWidget(refresh_btn)
        
        layout.addStretch()
        
    def _create_stat_label(self, icon, value):
        """Create a compact statistics label"""
        widget = QWidget()
        widget.setFixedHeight(45)
        layout = QVBoxLayout(widget)
        layout.setSpacing(1)
        layout.setContentsMargins(5, 5, 5, 5)
        widget.setStyleSheet("""
            QWidget {
                background-color: #0d121b;
                border: 1px solid #232d3d;
                border-radius: 6px;
            }
        """)
        
        value_label = QLabel(value)
        value_label.setStyleSheet("color: #22d3ee; font-size: 18px; font-weight: bold;")
        value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(value_label)
        
        name_label = QLabel(icon)
        name_label.setStyleSheet("color: #8492a6; font-size: 10px;")
        name_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(name_label)
        
        widget.value_label = value_label
        widget.name_label = name_label
        
        return widget
    
    def start_monitoring(self):
        if ROS2_INTROSPECTION_AVAILABLE:
            self.worker = RosStatusWorker()
            self.worker.status_updated.connect(self.update_status)
            self.worker.start()
    
    def update_status(self, status):
        if status['active']:
            self.status_dot.setStyleSheet("color: #2dd97a; font-size: 16px;")
            self.status_text.setText("ROS2 Active")
            self.status_text.setStyleSheet("color: #2dd97a; font-size: 12px;")
        else:
            self.status_dot.setStyleSheet("color: #ff3b5c; font-size: 16px;")
            self.status_text.setText("ROS2 Not Detected")
            self.status_text.setStyleSheet("color: #ff3b5c; font-size: 12px;")
        
        self.node_count_label.value_label.setText(str(status['node_count']))
        self.topic_count_label.value_label.setText(str(status['topic_count']))
        self.service_count_label.value_label.setText(str(status['service_count']))
        
        self.node_tree.clear()
        for node in status['nodes'][:10]:
            item = QTreeWidgetItem([node, "✓"])
            if 'servo' in node.lower():
                item.setForeground(1, QColor(100, 200, 255))
            elif 'moveit' in node.lower():
                item.setForeground(1, QColor(255, 200, 100))
            self.node_tree.addTopLevelItem(item)
        if len(status['nodes']) > 10:
            item = QTreeWidgetItem([f"... and {len(status['nodes']) - 10} more", ""])
            item.setForeground(0, QColor(100, 100, 120))
            self.node_tree.addTopLevelItem(item)
        
        self.topic_list.clear()
        for topic in status['topics'][:10]:
            self.topic_list.addItem(topic)
        if len(status['topics']) > 10:
            self.topic_list.addItem(f"... +{len(status['topics']) - 10} more")
    
    def force_refresh(self):
        if self.worker:
            status = self.worker.get_ros_status()
            self.update_status(status)
    
    def stop(self):
        if self.worker:
            self.worker.stop()
            self.worker.wait()


class SettingsDialog(QDialog):
    """Compact Settings dialog"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.settings = QSettings("NextupRobotics", "CobotHMI")
        self.setWindowTitle("Settings")
        self.setFixedSize(400, 450)  # Fixed compact size
        self.setStyleSheet("""
            QDialog {
                background-color: #090c12;
            }
            QLabel {
                color: #dbe4f0;
                font-size: 11px;
            }
            QGroupBox {
                color: #22d3ee;
                border: 1px solid #232d3d;
                border-radius: 4px;
                margin-top: 8px;
                font-weight: bold;
                font-size: 11px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px 0 4px;
            }
            QComboBox {
                background-color: #111826;
                color: #dbe4f0;
                border: 1px solid #232d3d;
                border-radius: 3px;
                padding: 4px;
                font-size: 11px;
                min-height: 22px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::item {
                padding: 3px;
            }
            QComboBox::item:selected {
                background-color: #232d3d;
            }
            QSlider::groove:horizontal {
                height: 3px;
                background: #232d3d;
                border-radius: 2px;
            }
            QSlider::handle:horizontal {
                background: #22d3ee;
                width: 12px;
                height: 12px;
                margin: -5px 0;
                border-radius: 6px;
            }
            QSlider::sub-page:horizontal {
                background: #22d3ee;
                border-radius: 2px;
            }
            QCheckBox {
                color: #dbe4f0;
                spacing: 6px;
                font-size: 11px;
            }
            QCheckBox::indicator {
                width: 14px;
                height: 14px;
                border: 2px solid #2a3441;
                border-radius: 3px;
                background-color: #0d121b;
            }
            QCheckBox::indicator:checked {
                background-color: #22d3ee;
                border-color: #67e8f9;
            }
            QPushButton {
                background-color: #182233;
                color: #dbe4f0;
                border: 1px solid #2a3441;
                border-radius: 3px;
                padding: 4px 12px;
                font-size: 11px;
                min-height: 22px;
            }
            QPushButton:hover {
                background-color: #2a3441;
            }
            QPushButton#primary {
                background-color: #123240;
                border-color: #1c4a57;
            }
            QPushButton#primary:hover {
                background-color: #14313d;
            }
            .hint-label {
                color: #606080;
                font-size: 9px;
                font-style: italic;
            }
        """)
        self.init_ui()
        self.load_settings()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(12, 12, 12, 12)
        
        # Theme - compact
        theme_group = QGroupBox("Theme")
        theme_layout = QVBoxLayout(theme_group)
        theme_layout.setSpacing(4)
        theme_layout.setContentsMargins(8, 8, 8, 8)
        
        self.theme_combo = QComboBox()
        self.theme_combo.addItems(["Dark", "Light"])
        self.theme_combo.currentTextChanged.connect(self.on_theme_changed)
        theme_layout.addWidget(self.theme_combo)
        layout.addWidget(theme_group)
        
        # Quality - compact
        quality_group = QGroupBox("STL Quality")
        quality_layout = QVBoxLayout(quality_group)
        quality_layout.setSpacing(4)
        quality_layout.setContentsMargins(8, 8, 8, 8)
        
        self.quality_combo = QComboBox()
        self.quality_combo.addItems(["Low 🚀", "Medium ⚡", "High ✨", "Original 🎨"])
        self.quality_combo.setCurrentIndex(1)
        quality_layout.addWidget(self.quality_combo)
        
        hint = QLabel("Low: fastest | Original: best quality")
        hint.setProperty("class", "hint-label")
        quality_layout.addWidget(hint)
        layout.addWidget(quality_group)
        
        # Camera - compact
        camera_group = QGroupBox("Camera")
        camera_layout = QVBoxLayout(camera_group)
        camera_layout.setSpacing(4)
        camera_layout.setContentsMargins(8, 8, 8, 8)
        
        # Distance
        dist_layout = QHBoxLayout()
        dist_layout.addWidget(QLabel("Distance:"))
        self.distance_slider = QSlider(Qt.Orientation.Horizontal)
        self.distance_slider.setRange(5, 30)
        self.distance_slider.setValue(15)
        self.distance_slider.valueChanged.connect(self.on_distance_changed)
        dist_layout.addWidget(self.distance_slider)
        self.distance_label = QLabel("1.5m")
        self.distance_label.setFixedWidth(35)
        dist_layout.addWidget(self.distance_label)
        camera_layout.addLayout(dist_layout)
        
        # Map size
        map_layout = QHBoxLayout()
        map_layout.addWidget(QLabel("Map:"))
        self.map_slider = QSlider(Qt.Orientation.Horizontal)
        self.map_slider.setRange(5, 30)
        self.map_slider.setValue(15)
        self.map_slider.valueChanged.connect(self.on_map_changed)
        map_layout.addWidget(self.map_slider)
        self.map_label = QLabel("1.5m")
        self.map_label.setFixedWidth(35)
        map_layout.addWidget(self.map_label)
        camera_layout.addLayout(map_layout)
        
        # Anti-aliasing
        aa_layout = QHBoxLayout()
        self.aa_checkbox = QCheckBox("Anti-Aliasing")
        aa_layout.addWidget(self.aa_checkbox)
        camera_layout.addLayout(aa_layout)
        layout.addWidget(camera_group)
        
        # Visualization - compact
        vis_group = QGroupBox("Visualization")
        vis_layout = QVBoxLayout(vis_group)
        vis_layout.setSpacing(4)
        vis_layout.setContentsMargins(8, 8, 8, 8)
        
        self.vis_checkbox = QCheckBox("Enable 3D View")
        self.vis_checkbox.setChecked(True)
        vis_layout.addWidget(self.vis_checkbox)
        
        hint2 = QLabel("Shows ROS2 status when disabled")
        hint2.setProperty("class", "hint-label")
        vis_layout.addWidget(hint2)
        layout.addWidget(vis_group)
        
        # Buttons - compact
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)
        
        apply_btn = QPushButton("Apply")
        apply_btn.setObjectName("primary")
        apply_btn.clicked.connect(self.apply_settings)
        btn_layout.addWidget(apply_btn)
        
        default_btn = QPushButton("Defaults")
        default_btn.clicked.connect(self.restore_defaults)
        btn_layout.addWidget(default_btn)
        
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        btn_layout.addWidget(close_btn)
        
        layout.addLayout(btn_layout)
        layout.addStretch()
    
    def load_settings(self):
        """Load settings from QSettings"""
        # Theme
        theme = self.settings.value("theme", "Dark")
        index = self.theme_combo.findText(theme)
        if index >= 0:
            self.theme_combo.setCurrentIndex(index)
        
        # STL Quality - handle both old and new format
        quality = self.settings.value("stl_quality", "medium")
        quality_map = {
            'low': 0,
            'medium': 1,
            'high': 2,
            'original': 3
        }
        # Handle the display text with icons
        for i in range(self.quality_combo.count()):
            text = self.quality_combo.itemText(i)
            if quality.lower() in text.lower():
                self.quality_combo.setCurrentIndex(i)
                break
        
        # Camera distance
        distance = self.settings.value("camera_distance", 15)
        try:
            distance = int(distance)
        except (ValueError, TypeError):
            distance = 15
        self.distance_slider.setValue(distance)
        self.update_distance_label()
        
        # Map size
        map_size = self.settings.value("map_size", 15)
        try:
            map_size = int(map_size)
        except (ValueError, TypeError):
            map_size = 15
        self.map_slider.setValue(map_size)
        self.update_map_label()
        
        # Anti-aliasing
        aa = self.settings.value("anti_aliasing", False)
        if isinstance(aa, str):
            aa = aa.lower() == "true"
        self.aa_checkbox.setChecked(bool(aa))
        
        # Visualization
        vis = self.settings.value("visualization_enabled", True)
        if isinstance(vis, str):
            vis = vis.lower() == "true"
        self.vis_checkbox.setChecked(bool(vis))
    
    def update_distance_label(self):
        val = self.distance_slider.value()
        self.distance_label.setText(f"{val/10:.1f}m")
    
    def update_map_label(self):
        val = self.map_slider.value()
        self.map_label.setText(f"{val/10:.1f}m")
    
    def on_theme_changed(self, theme):
        if self.parent:
            self.parent.apply_theme(theme)
    
    def on_distance_changed(self):
        self.update_distance_label()
    
    def on_map_changed(self):
        self.update_map_label()
    
    def apply_settings(self):
        # Theme
        theme = self.theme_combo.currentText()
        self.settings.setValue("theme", theme)
        if self.parent:
            self.parent.apply_theme(theme)
        
        # STL Quality - extract the actual quality from display text
        quality_text = self.quality_combo.currentText()
        if 'Low' in quality_text:
            quality = 'low'
        elif 'Medium' in quality_text:
            quality = 'medium'
        elif 'High' in quality_text:
            quality = 'high'
        else:
            quality = 'original'
        
        self.settings.setValue("stl_quality", quality)
        if self.parent and self.parent.viewer:
            self.parent.viewer.reload_meshes_with_quality(quality)
        
        # Camera distance
        distance = self.distance_slider.value()
        self.settings.setValue("camera_distance", distance)
        if self.parent and self.parent.viewer:
            self.parent.viewer.set_camera_distance(distance / 10.0)
        
        # Map size
        map_size = self.map_slider.value()
        self.settings.setValue("map_size", map_size)
        if self.parent and self.parent.viewer:
            self.parent.viewer.set_map_size(map_size / 10.0)
        
        # Anti-aliasing
        aa = self.aa_checkbox.isChecked()
        self.settings.setValue("anti_aliasing", aa)
        if self.parent and self.parent.viewer:
            self.parent.viewer.set_anti_aliasing(aa)
        
        # Visualization
        vis = self.vis_checkbox.isChecked()
        self.settings.setValue("visualization_enabled", vis)
        if self.parent:
            self.parent.toggle_visualization(vis)
        
        if self.parent:
            self.parent.statusBar().showMessage("✅ Settings applied")
        self.accept()
    
    def restore_defaults(self):
        self.theme_combo.setCurrentText("Dark")
        self.quality_combo.setCurrentIndex(1)  # Medium
        self.distance_slider.setValue(15)
        self.map_slider.setValue(15)
        self.aa_checkbox.setChecked(False)
        self.vis_checkbox.setChecked(True)
        self.update_distance_label()
        self.update_map_label()
        self.apply_settings()


class RobotHMI(QMainWindow):
    """Main HMI Application Window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Nextup Cobot HMI")
        self.setMinimumSize(1400, 800)
        
        # Settings
        self.settings = QSettings("NextupRobotics", "CobotHMI")
        
        # Initialize viewer as None first
        self.viewer = None
        self.ros_status_panel = None
        
        # Apply saved theme on startup
        theme = self.settings.value("theme", "Dark")
        self._pending_theme = theme
        self.apply_theme(theme)
        
        self.setStyleSheet(self.get_style_sheet())
        
        # Map display labels to actual joint names
        self.display_to_joint = {
            'J1': 'joint1',
            'J2': 'joint2',
            'J3': 'joint3',
            'J4': 'joint4',
            'J5': 'joint5',
            'J6': 'joint6'
        }
        self.joint_to_display = {v: k for k, v in self.display_to_joint.items()}
        
        self.joint_controls = {}
        self.cartesian_controls = {}
        self.active_motions = {}
        self.cartesian_linear_speed = 0.05
        self.cartesian_angular_speed = 0.3
        self.servo = None
        self.planner = None
        self.ros_worker = None
        self.current_hover_axis = None
        self.current_hover_direction = None
        self.settings_dialog = None
        self.visualization_enabled = True
        self.splitter = None
        
        # Initialize UI
        self.init_ui()
        self.init_ros()
    
    def get_style_sheet(self):
        """Global QSS for the app chrome. All colors live in theme.py."""
        theme = self.settings.value("theme", "Dark")
        return app_qss(theme)
    
    def apply_theme(self, theme):
        self.settings.setValue("theme", theme)
        self.setStyleSheet(self.get_style_sheet())
        if self.viewer is not None:
            self.viewer.apply_theme(theme)
    
    def toggle_visualization(self, enabled):
        self.visualization_enabled = enabled
        
        if self.viewer is None or self.ros_status_panel is None:
            return
        
        if hasattr(self, 'splitter') and self.splitter:
            if enabled:
                self.viewer.setVisible(True)
                self.ros_status_panel.setVisible(False)
                self.viewer.set_visualization_enabled(True)
                self.statusBar().showMessage("🔍 Visualization enabled")
                if hasattr(self, 'vis_toggle_btn'):
                    self.vis_toggle_btn.setText("🔍 Hide 3D")
            else:
                self.viewer.setVisible(False)
                self.ros_status_panel.setVisible(True)
                self.viewer.set_visualization_enabled(False)
                self.statusBar().showMessage("📊 Showing ROS2 status")
                if hasattr(self, 'vis_toggle_btn'):
                    self.vis_toggle_btn.setText("🔍 Show 3D")

    def init_ui(self):
        """Initialize the user interface"""
        self.create_menu_bar()
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)

        self.splitter = QSplitter(Qt.Orientation.Horizontal)
        left_panel = self.create_control_panel()
        self.splitter.addWidget(left_panel)

        # Create viewer
        self.viewer = Robot3DViewer(self)
        
        # Apply saved settings with type conversion
        distance = self.settings.value("camera_distance", 15)
        try:
            distance = float(distance) / 10.0
        except (ValueError, TypeError):
            distance = 1.5
        self.viewer.set_camera_distance(distance)
        
        map_size = self.settings.value("map_size", 15)
        try:
            map_size = float(map_size) / 10.0
        except (ValueError, TypeError):
            map_size = 1.5
        self.viewer.set_map_size(map_size)
        
        aa = self.settings.value("anti_aliasing", False)
        if isinstance(aa, str):
            aa = aa.lower() == "true"
        self.viewer.set_anti_aliasing(bool(aa))
        
        vis = self.settings.value("visualization_enabled", True)
        if isinstance(vis, str):
            vis = vis.lower() == "true"
        self.viewer.set_visualization_enabled(bool(vis))
        
        # Apply STL quality
        quality = self.settings.value("stl_quality", "medium")
        if isinstance(quality, str):
            quality = quality.lower()
        self.viewer.reload_meshes_with_quality(quality)
        
        if hasattr(self, '_pending_theme'):
            self.viewer.apply_theme(self._pending_theme)
        
        self.splitter.addWidget(self.viewer)
        
        # Create ROS status panel (initially hidden)
        self.ros_status_panel = RosStatusPanel(self)
        self.ros_status_panel.setVisible(False)
        self.splitter.addWidget(self.ros_status_panel)

        right_panel = self.create_preview_panel()
        self.splitter.addWidget(right_panel)

        self.splitter.setSizes([400, 1000, 220, 200])
        self.splitter.setHandleWidth(1)
        # Keep the Ghost/Arrow preview panel from being silently collapsed to 0px
        self.splitter.setCollapsible(3, False)

        main_layout.addWidget(self.splitter)

        self.viewer.joint_values_updated.connect(self.update_joint_display)

    def create_menu_bar(self):
        menubar = self.menuBar()
        
        file_menu = menubar.addMenu("File")
        exit_action = file_menu.addAction("Exit")
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        
        view_menu = menubar.addMenu("View")
        reset_action = view_menu.addAction("Reset Camera")
        reset_action.setShortcut("R")
        reset_action.triggered.connect(self.reset_camera)
        
        view_menu.addSeparator()
        
        vis_action = view_menu.addAction("Toggle Visualization")
        vis_action.setShortcut("Ctrl+V")
        vis_action.triggered.connect(self.toggle_visualization_action)
        
        settings_menu = menubar.addMenu("Settings")
        settings_action = settings_menu.addAction("Preferences...")
        settings_action.setShortcut("Ctrl+,")
        settings_action.triggered.connect(self.open_settings)
        
        settings_menu.addSeparator()
        
        theme_menu = settings_menu.addMenu("Theme")
        dark_action = theme_menu.addAction("Dark")
        dark_action.triggered.connect(lambda: self.apply_theme("Dark"))
        light_action = theme_menu.addAction("Light")
        light_action.triggered.connect(lambda: self.apply_theme("Light"))
        
        safety_menu = menubar.addMenu("Safety")
        stop_action = safety_menu.addAction("Emergency Stop")
        stop_action.setShortcut("Space")
        stop_action.triggered.connect(self.emergency_stop)
        
        help_menu = menubar.addMenu("Help")
        about_action = help_menu.addAction("About")
        about_action.triggered.connect(self.show_about)

    def toggle_visualization_action(self):
        self.visualization_enabled = not self.visualization_enabled
        self.toggle_visualization(self.visualization_enabled)

    def open_settings(self):
        if self.settings_dialog is None:
            self.settings_dialog = SettingsDialog(self)
        self.settings_dialog.show()
        self.settings_dialog.raise_()
        self.settings_dialog.activateWindow()

    def create_control_panel(self):
        panel = QWidget()
        panel.setStyleSheet("QWidget { background-color: transparent; }")
        panel.setFixedWidth(420)
        
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(15)

        title = QLabel("\u2921  MOTION CONSOLE")
        title.setStyleSheet(section_title_qss(size=18))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        speed_widget = QWidget()
        speed_layout = QHBoxLayout(speed_widget)
        speed_layout.setContentsMargins(5, 10, 5, 10)
        
        speed_label = QLabel("Speed:")
        speed_label.setStyleSheet(f"color: {Palette.TEXT_MUTED}; font-size: 12px;")
        speed_layout.addWidget(speed_label)
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 10)
        self.speed_slider.setValue(5)
        self.speed_slider.setTickInterval(1)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.setStyleSheet(slider_qss())
        self.speed_slider.valueChanged.connect(self.update_speed)
        speed_layout.addWidget(self.speed_slider)
        
        self.speed_label = QLabel("0.5x")
        self.speed_label.setFont(mono_font(12, bold=True))
        self.speed_label.setStyleSheet(f"color: {Palette.ACCENT}; min-width: 40px;")
        speed_layout.addWidget(self.speed_label)
        layout.addWidget(speed_widget)

        # Tab look & feel comes from the global app_qss() - no per-widget override needed.
        tabs = QTabWidget()
        joint_tab = self.create_joint_tab()
        cart_tab = self.create_cartesian_tab()
        tabs.addTab(joint_tab, "Joints")
        tabs.addTab(cart_tab, "Cartesian")
        layout.addWidget(tabs)

        vis_widget = QWidget()
        vis_layout = QHBoxLayout(vis_widget)
        vis_layout.setContentsMargins(0, 5, 0, 5)
        vis_layout.setSpacing(8)
        
        self.vis_toggle_btn = QPushButton("\U0001F441  Hide 3D")
        self.vis_toggle_btn.setStyleSheet(button_qss(
            bg="#12212b", fg=Palette.ACCENT_HOVER, border="#1c4a57",
            hover_bg="#14313d", hover_border=Palette.ACCENT,
            radius=6, padding="5px 12px", font_size=11,
        ))
        self.vis_toggle_btn.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.vis_toggle_btn.clicked.connect(self.toggle_visualization_action)
        vis_layout.addWidget(self.vis_toggle_btn)
        
        settings_btn = QPushButton("\u2699  Settings")
        settings_btn.setStyleSheet(button_qss(radius=6, padding="5px 12px", font_size=11))
        settings_btn.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        settings_btn.clicked.connect(self.open_settings)
        vis_layout.addWidget(settings_btn)
        
        layout.addWidget(vis_widget)

        estop = QPushButton("\u26A0  EMERGENCY STOP")
        estop.setStyleSheet(button_qss(
            bg=Palette.DANGER_DIM, fg=Palette.DANGER, border=Palette.DANGER,
            hover_bg="#5a1420", hover_border=Palette.DANGER_HOVER,
            pressed_bg="#2a0a10", radius=8, padding="12px",
            font_size=14, bold=True, letter_spacing=2,
        ))
        estop.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        estop.clicked.connect(self.emergency_stop)
        layout.addWidget(estop)

        status_widget = QWidget()
        status_layout = QHBoxLayout(status_widget)
        status_layout.setContentsMargins(5, 10, 5, 10)
        
        status_dot = QLabel("●")
        status_dot.setStyleSheet(f"color: {Palette.SUCCESS}; font-size: 12px;")
        status_layout.addWidget(status_dot)
        
        self.status_label = QLabel("Initializing...")
        self.status_label.setStyleSheet(f"color: {Palette.TEXT_MUTED}; font-size: 11px;")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        layout.addWidget(status_widget)

        version = QLabel("Nextup Cobot HMI v1.0")
        version.setStyleSheet(f"color: {Palette.TEXT_DIM}; font-size: 10px;")
        version.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(version)
        layout.addStretch()

        return panel

    def create_joint_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 15, 10, 15)
        layout.setSpacing(8)

        header = QLabel("JOINT POSITIONS")
        header.setStyleSheet("color: #22d3ee; font-weight: bold; font-size: 12px;")
        layout.addWidget(header)

        info = QLabel("Hover + or - to preview, press and hold to move joint")
        info.setStyleSheet("color: #4a5568; font-size: 10px; font-style: italic;")
        layout.addWidget(info)

        joint_display_labels = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        for idx, (display_label, joint_name) in enumerate(zip(joint_display_labels, joint_names)):
            ctrl = JointControlWidget(display_label, idx)
            ctrl.actual_joint_name = joint_name
            ctrl.hover_enter.connect(self.on_joint_hover_enter)
            ctrl.hover_leave.connect(self.on_joint_hover_leave)
            ctrl.joint_velocity.connect(self.on_joint_velocity)
            ctrl.joint_stop.connect(self.on_joint_stop)
            self.joint_controls[joint_name] = ctrl
            layout.addWidget(ctrl)

        layout.addStretch()
        return tab

    def create_cartesian_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 15, 10, 15)
        layout.setSpacing(8)

        header = QLabel("CARTESIAN SPACE")
        header.setStyleSheet("color: #22d3ee; font-weight: bold; font-size: 12px;")
        layout.addWidget(header)

        info = QLabel("Hover + or - to plan motion" + (" (MoveIt)" if MOVEIT_AVAILABLE else " (disabled)"))
        info.setStyleSheet("color: #4a5568; font-size: 10px; font-style: italic;")
        layout.addWidget(info)

        trans_group = QGroupBox("Translation")
        trans_layout = QVBoxLayout(trans_group)
        trans_layout.setSpacing(5)
        
        for axis, label in [('x', 'X'), ('y', 'Y'), ('z', 'Z')]:
            ctrl = CartesianControlWidget(axis, label)
            ctrl.hover_enter.connect(self.on_cartesian_hover_enter)
            ctrl.hover_leave.connect(self.on_cartesian_hover_leave)
            ctrl.cartesian_command.connect(self.on_cartesian_velocity)
            ctrl.cartesian_stop.connect(self.on_cartesian_stop)
            self.cartesian_controls[axis] = ctrl
            trans_layout.addWidget(ctrl)
        layout.addWidget(trans_group)

        rot_group = QGroupBox("Rotation")
        rot_layout = QVBoxLayout(rot_group)
        rot_layout.setSpacing(5)
        
        for axis, label in [('roll', 'Roll'), ('pitch', 'Pitch'), ('yaw', 'Yaw')]:
            ctrl = CartesianControlWidget(axis, label)
            ctrl.hover_enter.connect(self.on_cartesian_hover_enter)
            ctrl.hover_leave.connect(self.on_cartesian_hover_leave)
            ctrl.cartesian_command.connect(self.on_cartesian_velocity)
            ctrl.cartesian_stop.connect(self.on_cartesian_stop)
            self.cartesian_controls[axis] = ctrl
            rot_layout.addWidget(ctrl)
        layout.addWidget(rot_group)

        layout.addStretch()
        return tab

    def create_preview_panel(self):
        panel = QWidget()
        panel.setStyleSheet("background-color: transparent;")
        panel.setFixedWidth(200)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        title = QLabel("HOVER PREVIEW")
        title.setStyleSheet(section_title_qss(size=13))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        checkbox_style = checkbox_qss()

        self.ghost_checkbox = QCheckBox("👻 Ghost Preview")
        self.ghost_checkbox.setStyleSheet(checkbox_style)
        self.ghost_checkbox.setChecked(False)
        self.ghost_checkbox.toggled.connect(self.on_ghost_toggle)
        layout.addWidget(self.ghost_checkbox)

        ghost_hint = QLabel("Translucent robot at planned pose")
        ghost_hint.setStyleSheet(f"color: {Palette.TEXT_DIM}; font-size: 9px; font-style: italic; padding-left: 24px;")
        ghost_hint.setWordWrap(True)
        layout.addWidget(ghost_hint)

        self.arrow_checkbox = QCheckBox("➡ Arrow Indicator")
        self.arrow_checkbox.setStyleSheet(checkbox_style)
        self.arrow_checkbox.setChecked(False)
        self.arrow_checkbox.toggled.connect(self.on_arrow_toggle)
        layout.addWidget(self.arrow_checkbox)

        arrow_hint = QLabel("Arrow at joint/axis in motion")
        arrow_hint.setStyleSheet(f"color: {Palette.TEXT_DIM}; font-size: 9px; font-style: italic; padding-left: 24px;")
        arrow_hint.setWordWrap(True)
        layout.addWidget(arrow_hint)

        layout.addStretch()
        return panel

    def on_ghost_toggle(self, checked):
        if self.viewer:
            self.viewer.set_ghost_enabled(checked)

    def on_arrow_toggle(self, checked):
        if self.viewer:
            self.viewer.set_arrow_enabled(checked)

    def init_ros(self):
        self.ros_worker = RosWorker()
        self.ros_worker.node_initialized.connect(self.on_node_ready)
        self.ros_worker.start()
        self.status_label.setText("Connecting to ROS...")

    def on_node_ready(self, node):
        self.servo = ServoController(node)
        self.status_label.setText("Connected - Servo Active")

        # Drive the joint readouts straight from /joint_states. (The viewer's
        # joint_values_updated signal is still connected above for whenever
        # the 3D viewer starts emitting it, but it never does today - so this
        # is the actual source of the numbers you see next to each joint.)
        self.joint_state_sub = node.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10
        )

        if MOVEIT_AVAILABLE:
            try:
                self.planner = MoveItPlanner(node, group='robot_manipulator')
                self.planner.planning_done.connect(self.on_planning_done)
                self.planner.trajectory_ready.connect(self.on_trajectory_ready)
                self.planner.progress_update.connect(self.on_planning_progress)
                if self.planner.available:
                    self.status_label.setText("Connected - Servo & Planning Active")
                else:
                    self.status_label.setText("Connected - Servo Active (Cartesian planning disabled, joint preview OK)")
            except Exception as e:
                print(f"MoveIt init error: {e}")
                self.status_label.setText("Connected - Servo Active (Planning error)")
        else:
            self.status_label.setText("Connected - Servo Active (MoveIt not installed)")

        self.viewer.setup_tf(node)
        print("✓ ROS2 initialized")

    def _on_joint_state(self, msg):
        """Live feedback from the robot/simulator - updates the value readout
        next to each joint's +/- buttons."""
        self.update_joint_display(msg.name, msg.position)

    _JOINT_PREVIEW_OFFSET = 0.3

    def on_joint_hover_enter(self, display_name, direction):
        joint_name = self.display_to_joint.get(display_name, display_name)
        preview_wanted = self.viewer and (self.viewer.ghost_enabled or self.viewer.arrow_enabled)
        if self.planner and preview_wanted:
            self.current_hover_axis = joint_name
            self.current_hover_direction = direction
            offset = self._JOINT_PREVIEW_OFFSET
            if direction == 'minus':
                offset = -offset
            self.planner.plan_joint_offset(joint_name, offset)

        direction_text = '+' if direction == 'plus' else '-'
        if preview_wanted:
            self.statusBar().showMessage(f"Joint {display_name} - {direction_text}")
        else:
            self.statusBar().showMessage(f"Joint {display_name} - {direction_text} (enable Ghost/Arrow preview to see it)")

    def on_joint_hover_leave(self, display_name):
        if self.viewer:
            self.viewer.clear_motion_preview()
        self.current_hover_axis = None
        self.current_hover_direction = None
        self.statusBar().showMessage("Ready")

    def on_joint_velocity(self, display_name, velocity):
        if not self.servo:
            return
        
        joint_name = self.display_to_joint.get(display_name, display_name)
        
        if joint_name not in self.joint_controls:
            print(f"⚠ Unknown joint: {joint_name}")
            return
            
        idx = self.joint_controls[joint_name].joint_index

        if joint_name in self.active_motions:
            self.active_motions[joint_name].stop()
            self.active_motions[joint_name].deleteLater()

        # JointJog is a streaming command - keep publishing while the button is held
        self.servo.publish_joint_velocity_by_index(idx, velocity)
        timer = QTimer()
        timer.setInterval(50)
        timer.timeout.connect(lambda: self.servo.publish_joint_velocity_by_index(idx, velocity))
        timer.start()
        self.active_motions[joint_name] = timer

    def on_joint_stop(self, display_name):
        joint_name = self.display_to_joint.get(display_name, display_name)
        
        if joint_name in self.active_motions:
            self.active_motions[joint_name].stop()
            self.active_motions[joint_name].deleteLater()
            del self.active_motions[joint_name]
        
        if self.servo and joint_name in self.joint_controls:
            idx = self.joint_controls[joint_name].joint_index
            self.servo.publish_joint_velocity_by_index(idx, 0.0)

    def on_cartesian_hover_enter(self, axis, direction):
        if not self.planner or not self.planner.available:
            self.statusBar().showMessage("MoveIt planning not available")
            return

        preview_wanted = self.viewer and (self.viewer.ghost_enabled or self.viewer.arrow_enabled)
        if not preview_wanted:
            direction_text = '+' if direction == 'plus' else '-'
            self.statusBar().showMessage(f"{axis} {direction_text} (enable Ghost/Arrow preview to see it)")
            return

        self.current_hover_axis = axis
        self.current_hover_direction = direction
            
        offset = 0.1 if axis in ['x', 'y', 'z'] else 0.2
        if direction == 'minus':
            offset = -offset
            
        self.planner.plan_cartesian_offset(axis, offset)
        direction_text = '+' if direction == 'plus' else '-'
        self.statusBar().showMessage(f"Planning {axis} {direction_text} offset...")

    def on_cartesian_hover_leave(self, axis):
        self.current_hover_axis = None
        self.current_hover_direction = None
        if self.viewer:
            self.viewer.clear_motion_preview()
        self.statusBar().showMessage("Ready")

    # Which linear/angular component of a TwistStamped each Cartesian axis drives
    _CARTESIAN_LINEAR_AXES = {'x': 0, 'y': 1, 'z': 2}
    _CARTESIAN_ANGULAR_AXES = {'roll': 0, 'pitch': 1, 'yaw': 2}
    _CARTESIAN_AXES = set(_CARTESIAN_LINEAR_AXES) | set(_CARTESIAN_ANGULAR_AXES)

    def _cartesian_twist_for(self, axis, direction_sign):
        linear = [0.0, 0.0, 0.0]
        angular = [0.0, 0.0, 0.0]
        if axis in self._CARTESIAN_LINEAR_AXES:
            linear[self._CARTESIAN_LINEAR_AXES[axis]] = direction_sign * self.cartesian_linear_speed
        elif axis in self._CARTESIAN_ANGULAR_AXES:
            angular[self._CARTESIAN_ANGULAR_AXES[axis]] = direction_sign * self.cartesian_angular_speed
        return tuple(linear), tuple(angular)

    def on_cartesian_velocity(self, axis, direction_sign):
        if not self.servo:
            return
        if axis not in self._CARTESIAN_AXES:
            print(f"⚠ Unknown Cartesian axis: {axis}")
            return

        linear, angular = self._cartesian_twist_for(axis, direction_sign)

        key = f'cart_{axis}'
        if key in self.active_motions:
            self.active_motions[key].stop()
            self.active_motions[key].deleteLater()

        # TwistStamped is a streaming command - keep publishing while the button is held
        self.servo.publish_twist(linear, angular)
        timer = QTimer()
        timer.setInterval(50)
        timer.timeout.connect(lambda: self.servo.publish_twist(linear, angular))
        timer.start()
        self.active_motions[key] = timer

        direction_text = '+' if direction_sign > 0 else '-'
        self.statusBar().showMessage(f"Jogging {axis} {direction_text}")

    def on_cartesian_stop(self, axis):
        key = f'cart_{axis}'
        if key in self.active_motions:
            self.active_motions[key].stop()
            self.active_motions[key].deleteLater()
            del self.active_motions[key]

        if self.servo and axis in self._CARTESIAN_AXES:
            self.servo.publish_twist()

        self.statusBar().showMessage("Ready")

    def on_trajectory_ready(self, display_trajectory, axis, direction):
        if self.current_hover_axis == axis and self.current_hover_direction == direction:
            if self.viewer:
                kind = 'cartesian' if axis in self._CARTESIAN_AXES else 'joint'
                self.viewer.show_motion_preview(kind, axis, direction, display_trajectory)

    def on_planning_done(self, success, message):
        if success:
            self.statusBar().showMessage(f"✓ {message}")
        else:
            self.statusBar().showMessage(f"✗ {message}")
            if self.viewer:
                self.viewer.clear_motion_preview()

    def on_planning_progress(self, progress):
        pass

    def update_speed(self, value):
        speed = value / 10.0
        self.speed_label.setText(f"{speed:.1f}x")
        for ctrl in self.joint_controls.values():
            ctrl.velocity = 0.5 * speed
        self.cartesian_linear_speed = 0.05 * speed
        self.cartesian_angular_speed = 0.3 * speed

    def update_joint_display(self, joint_names, positions):
        for name, pos in zip(joint_names, positions):
            if name in self.joint_controls:
                self.joint_controls[name].update_value(pos)

    def reset_camera(self):
        if self.viewer:
            self.viewer.reset_camera()
            self.statusBar().showMessage("Camera reset to default position")

    def emergency_stop(self):
        for timer in self.active_motions.values():
            timer.stop()
            timer.deleteLater()
        self.active_motions.clear()
        
        if self.servo:
            self.servo.publish_stop_all()
        
        self.statusBar().showMessage("⚠ EMERGENCY STOP")
        QMessageBox.warning(self, "Emergency Stop", "All joints have been stopped!")

    def show_about(self):
        QMessageBox.about(self, "About Nextup Cobot HMI",
            "<h2>Nextup Cobot HMI</h2>"
            "<p>Version 1.0</p>"
            "<p>Robot control with MoveIt planning</p>"
            "<p>© Nextup Robotics</p>"
            "<br>"
            "<b>Features:</b><br>"
            "• Dark/Light theme<br>"
            "• STL quality: Low, Medium, High, Original<br>"
            "• Camera distance & map size<br>"
            "• Anti-aliasing<br>"
            "• Visualization toggle (shows ROS2 status)<br>"
            "• Shortcuts: Ctrl+V, Ctrl+,, R, Space"
        )

    def closeEvent(self, event):
        self.emergency_stop()
        if self.ros_worker:
            self.ros_worker.stop()
            self.ros_worker.wait()
        event.accept()