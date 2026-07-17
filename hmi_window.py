"""
Main HMI Window for Nextup Cobot HMI
"""
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QSplitter, QLabel, QPushButton, QSlider,
    QStatusBar, QMessageBox, QGroupBox, QCheckBox,
    QMenu, QDialog, QDialogButtonBox, QLineEdit, QComboBox,
    QFormLayout, QListWidget, QListWidgetItem, QPushButton,
    QHBoxLayout, QFrame, QScrollArea
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QSettings
from PyQt6.QtGui import QAction, QKeySequence
from joint_control_widget import JointControlWidget
from cartesian_control_widget import CartesianControlWidget
from viewer_widget import Robot3DViewer
from servo_controller import ServoController

# Try to import MoveIt planner
try:
    from moveit_planner import MoveItPlanner
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False
    print("⚠ MoveIt planner not available")

from ros_worker import RosWorker


class CameraPresetDialog(QDialog):
    """Dialog for saving and managing camera presets"""
    
    def __init__(self, viewer, parent=None):
        super().__init__(parent)
        self.viewer = viewer
        self.settings = QSettings("NextupRobotics", "CobotHMI")
        self.setWindowTitle("Camera Presets")
        self.setMinimumWidth(400)
        self.setMinimumHeight(300)
        self.setStyleSheet("""
            QDialog {
                background-color: #0a0a1a;
            }
            QLabel {
                color: #c0c0e0;
            }
            QLineEdit {
                background-color: #1a1a3a;
                color: #c0c0e0;
                border: 1px solid #2a2a4a;
                border-radius: 4px;
                padding: 6px;
            }
            QListWidget {
                background-color: #1a1a3a;
                color: #c0c0e0;
                border: 1px solid #2a2a4a;
                border-radius: 4px;
                padding: 4px;
            }
            QListWidget::item:selected {
                background-color: #2a2a4a;
            }
            QPushButton {
                background-color: #2a2a3e;
                color: #c0c0e0;
                border: 1px solid #3a3a5e;
                border-radius: 4px;
                padding: 6px 12px;
            }
            QPushButton:hover {
                background-color: #3a3a5e;
            }
            QPushButton:pressed {
                background-color: #4a4a6e;
            }
            QPushButton#danger {
                background-color: #4a1a1a;
                border-color: #6a2a2a;
            }
            QPushButton#danger:hover {
                background-color: #5a2a2a;
            }
            QPushButton#primary {
                background-color: #1a2a4a;
                border-color: #2a4a6a;
            }
            QPushButton#primary:hover {
                background-color: #2a3a5a;
            }
        """)
        self.init_ui()
        self.load_presets()

    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # Title
        title = QLabel("📷 Camera Presets")
        title.setStyleSheet("font-size: 16px; font-weight: bold; color: #6a6aae; padding: 8px 0;")
        layout.addWidget(title)
        
        # Preset list
        list_label = QLabel("Saved Presets:")
        layout.addWidget(list_label)
        
        self.preset_list = QListWidget()
        self.preset_list.itemDoubleClicked.connect(self.apply_preset)
        layout.addWidget(self.preset_list)
        
        # Preset management buttons
        btn_layout = QHBoxLayout()
        
        self.apply_btn = QPushButton("Apply")
        self.apply_btn.clicked.connect(self.apply_selected_preset)
        btn_layout.addWidget(self.apply_btn)
        
        self.save_btn = QPushButton("Save Current")
        self.save_btn.clicked.connect(self.save_current_preset)
        btn_layout.addWidget(self.save_btn)
        
        self.delete_btn = QPushButton("Delete")
        self.delete_btn.setObjectName("danger")
        self.delete_btn.clicked.connect(self.delete_selected_preset)
        btn_layout.addWidget(self.delete_btn)
        
        layout.addLayout(btn_layout)
        
        # Save current preset dialog area
        save_frame = QFrame()
        save_frame.setStyleSheet("""
            QFrame {
                background-color: #141428;
                border: 1px solid #2a2a4a;
                border-radius: 4px;
                padding: 10px;
                margin-top: 10px;
            }
        """)
        save_layout = QFormLayout(save_frame)
        
        self.preset_name_edit = QLineEdit()
        self.preset_name_edit.setPlaceholderText("Enter preset name...")
        save_layout.addRow("Preset Name:", self.preset_name_edit)
        
        self.save_current_btn = QPushButton("Save Preset")
        self.save_current_btn.setObjectName("primary")
        self.save_current_btn.clicked.connect(self.save_current_preset)
        save_layout.addRow("", self.save_current_btn)
        
        layout.addWidget(save_frame)
        
        # Close button
        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        button_box.rejected.connect(self.accept)
        layout.addWidget(button_box)

    def load_presets(self):
        """Load saved presets from QSettings"""
        self.preset_list.clear()
        presets = self.settings.value("camera_presets", {})
        if isinstance(presets, dict):
            for name in sorted(presets.keys()):
                self.preset_list.addItem(name)
        else:
            # Handle legacy format
            self.settings.remove("camera_presets")
            self.settings.setValue("camera_presets", {})

    def save_current_preset(self):
        """Save current camera position as a preset"""
        name = self.preset_name_edit.text().strip()
        if not name:
            # Use default name with timestamp
            from datetime import datetime
            name = f"View_{datetime.now().strftime('%H:%M:%S')}"
        
        if not self.viewer:
            return
        
        # Get current camera pose
        camera = self.viewer.renderer.GetActiveCamera()
        preset_data = {
            'focal_point': list(camera.GetFocalPoint()),
            'position': list(camera.GetPosition()),
            'view_up': list(camera.GetViewUp()),
            'clipping_range': list(camera.GetClippingRange()),
        }
        
        # Save to settings
        presets = self.settings.value("camera_presets", {})
        if not isinstance(presets, dict):
            presets = {}
        presets[name] = preset_data
        self.settings.setValue("camera_presets", presets)
        
        # Update list
        self.load_presets()
        self.preset_name_edit.clear()
        
        # Select the new item
        items = self.preset_list.findItems(name, Qt.MatchFlag.MatchExactly)
        if items:
            self.preset_list.setCurrentItem(items[0])
        
        self.parent().statusBar().showMessage(f"✅ Camera preset '{name}' saved")

    def apply_preset(self, item):
        """Apply a camera preset by name"""
        name = item.text() if isinstance(item, QListWidgetItem) else item
        self.apply_preset_by_name(name)

    def apply_preset_by_name(self, name):
        """Apply camera preset by name"""
        if not self.viewer:
            return
        
        presets = self.settings.value("camera_presets", {})
        if not isinstance(presets, dict):
            return
        
        preset_data = presets.get(name)
        if not preset_data:
            return
        
        camera = self.viewer.renderer.GetActiveCamera()
        camera.SetFocalPoint(*preset_data['focal_point'])
        camera.SetPosition(*preset_data['position'])
        camera.SetViewUp(*preset_data['view_up'])
        camera.SetClippingRange(*preset_data['clipping_range'])
        self.viewer.renderer.ResetCameraClippingRange()
        self.viewer.vtk_widget.GetRenderWindow().Render()
        
        self.parent().statusBar().showMessage(f"📷 Applied preset: {name}")

    def apply_selected_preset(self):
        """Apply the currently selected preset"""
        current_item = self.preset_list.currentItem()
        if current_item:
            self.apply_preset(current_item)

    def delete_selected_preset(self):
        """Delete the currently selected preset"""
        current_item = self.preset_list.currentItem()
        if not current_item:
            return
        
        name = current_item.text()
        reply = QMessageBox.question(
            self, "Delete Preset",
            f"Delete camera preset '{name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            presets = self.settings.value("camera_presets", {})
            if isinstance(presets, dict) and name in presets:
                del presets[name]
                self.settings.setValue("camera_presets", presets)
                self.load_presets()
                self.parent().statusBar().showMessage(f"🗑️ Deleted preset: {name}")


class RobotHMI(QMainWindow):
    """Main HMI Application Window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Nextup Cobot HMI")
        self.setMinimumSize(1400, 800)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0a0a1a;
            }
            QMenuBar {
                background-color: #141425;
                color: #a0a0c0;
                border-bottom: 1px solid #2a2a4a;
            }
            QMenuBar::item {
                background-color: transparent;
                padding: 6px 12px;
            }
            QMenuBar::item:selected {
                background-color: #2a2a4a;
                color: #ffffff;
            }
            QMenu {
                background-color: #141425;
                color: #a0a0c0;
                border: 1px solid #2a2a4a;
            }
            QMenu::item:selected {
                background-color: #2a2a4a;
                color: #ffffff;
            }
            QTabWidget::pane {
                background-color: #0a0a1a;
                border: 1px solid #2a2a4a;
                border-radius: 4px;
            }
            QTabBar::tab {
                background-color: #141425;
                color: #a0a0c0;
                padding: 8px 20px;
                margin: 2px;
                border: 1px solid #2a2a4a;
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #2a2a4a;
                color: #ffffff;
                border-bottom: 2px solid #6a6aae;
            }
            QTabBar::tab:hover {
                background-color: #1a1a3a;
            }
            QLabel {
                color: #c0c0e0;
            }
            QGroupBox {
                color: #6a6aae;
                border: 1px solid #2a2a4a;
                border-radius: 4px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QStatusBar {
                background-color: #0a0a1a;
                color: #8080a0;
                border-top: 1px solid #1a1a3a;
            }
        """)

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
        # Cartesian jog speeds. command_in_type is "speed_units" in the servo
        # config, meaning these are raw m/s and rad/s -- NOT the -1..1
        # unitless range that scale.linear/scale.rotational would apply to.
        self.cartesian_linear_speed = 0.05    # m/s
        self.cartesian_angular_speed = 0.3    # rad/s
        self.viewer = None
        self.servo = None
        self.planner = None
        self.ros_worker = None
        self.current_hover_axis = None
        self.current_hover_direction = None
        self.camera_preset_dialog = None

        # Hover-preview debounce: sweeping the mouse across several jog
        # buttons used to fire a full FK->IK planning round-trip (plus a
        # ghost-actor rebuild + animation timer) on EVERY momentary
        # hover-enter, which is what caused CPU to spike when hovering
        # quickly. Now we wait for the pointer to settle on one button for
        # _HOVER_DEBOUNCE_MS before actually kicking off a plan; a hover
        # that moves on before that just gets cancelled, like a tooltip.
        self._hover_debounce_timer = QTimer()
        self._hover_debounce_timer.setSingleShot(True)
        self._hover_debounce_timer.timeout.connect(self._fire_pending_hover_plan)
        self._pending_hover_plan = None  # callable to run once settled
        
        # Settings for camera presets
        self.settings = QSettings("NextupRobotics", "CobotHMI")

        self.init_ui()
        self.init_ros()

    def init_ui(self):
        """Initialize the user interface"""
        # Menu
        self.create_menu_bar()
        
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)

        # Splitter
        splitter = QSplitter(Qt.Orientation.Horizontal)
        left_panel = self.create_control_panel()
        splitter.addWidget(left_panel)

        self.viewer = Robot3DViewer(self)
        splitter.addWidget(self.viewer)

        right_panel = self.create_preview_panel()
        splitter.addWidget(right_panel)

        splitter.setSizes([400, 1000, 220])
        splitter.setHandleWidth(1)

        main_layout.addWidget(splitter)

        # Connect viewer joint updates to UI
        self.viewer.joint_values_updated.connect(self.update_joint_display)

    def create_menu_bar(self):
        """Create menu bar"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("File")
        exit_action = menubar.addAction("Exit")
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # View menu - Enhanced with camera controls
        view_menu = menubar.addMenu("View")
        
        # Camera submenu
        camera_menu = view_menu.addMenu("📷 Camera")
        
        # Reset camera
        reset_action = camera_menu.addAction("Reset Camera")
        reset_action.setShortcut("R")
        reset_action.triggered.connect(self.reset_camera)
        
        camera_menu.addSeparator()
        
        # Preset views
        preset_views = {
            "Top View": (0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            "Front View": (0.0, -1.5, 0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.0),
            "Side View": (1.5, 0.0, 0.5, 0.0, 0.0, 0.2, 0.0, 1.0, 0.0),
            "Back View": (0.0, 1.5, 0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.0),
            "Left View": (-1.5, 0.0, 0.5, 0.0, 0.0, 0.2, 0.0, 1.0, 0.0),
            "Right View": (1.5, 0.0, 0.5, 0.0, 0.0, 0.2, 0.0, 1.0, 0.0),
            "Isometric": (1.5, -1.5, 1.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.0),
            "Close Up EE": (0.1, -0.5, 0.4, 0.0, 0.0, 0.3, 0.0, 0.0, 1.0),
        }
        
        for name, params in preset_views.items():
            action = camera_menu.addAction(name)
            action.triggered.connect(lambda checked, p=params: self.set_camera_view(*p))
        
        camera_menu.addSeparator()
        
        # Load saved presets dynamically
        self.preset_menu = camera_menu.addMenu("📂 Saved Presets")
        self.update_preset_menu()
        
        camera_menu.addSeparator()
        
        # Manage presets
        manage_action = camera_menu.addAction("⚙️ Manage Presets...")
        manage_action.triggered.connect(self.open_camera_preset_dialog)
        
        # Save current as default
        camera_menu.addSeparator()
        set_default_action = camera_menu.addAction("⭐ Set as Default View")
        set_default_action.triggered.connect(self.set_default_view)
        
        restore_default_action = camera_menu.addAction("↩️ Restore Default View")
        restore_default_action.triggered.connect(self.restore_default_view)
        
        view_menu.addSeparator()
        
        # Toggle debug overlay
        debug_action = view_menu.addAction("Toggle Debug Overlay")
        debug_action.setShortcut("Ctrl+D")
        debug_action.triggered.connect(self.toggle_debug_overlay)
        
        # Safety menu
        safety_menu = menubar.addMenu("Safety")
        stop_action = safety_menu.addAction("Emergency Stop")
        stop_action.setShortcut("Space")
        stop_action.triggered.connect(self.emergency_stop)
        
        # Help menu
        help_menu = menubar.addMenu("Help")
        about_action = help_menu.addAction("About")
        about_action.triggered.connect(self.show_about)

    def update_preset_menu(self):
        """Update the saved presets submenu"""
        self.preset_menu.clear()
        presets = self.settings.value("camera_presets", {})
        if not isinstance(presets, dict):
            presets = {}
        
        if not presets:
            no_presets = self.preset_menu.addAction("No saved presets")
            no_presets.setEnabled(False)
            return
        
        for name in sorted(presets.keys()):
            action = self.preset_menu.addAction(name)
            action.triggered.connect(lambda checked, n=name: self.apply_camera_preset(n))

    def apply_camera_preset(self, name):
        """Apply a saved camera preset"""
        if not self.viewer:
            return
        
        presets = self.settings.value("camera_presets", {})
        if not isinstance(presets, dict):
            return
        
        preset_data = presets.get(name)
        if not preset_data:
            return
        
        camera = self.viewer.renderer.GetActiveCamera()
        camera.SetFocalPoint(*preset_data['focal_point'])
        camera.SetPosition(*preset_data['position'])
        camera.SetViewUp(*preset_data['view_up'])
        camera.SetClippingRange(*preset_data['clipping_range'])
        self.viewer.renderer.ResetCameraClippingRange()
        self.viewer.vtk_widget.GetRenderWindow().Render()
        
        self.statusBar().showMessage(f"📷 Applied preset: {name}")

    def set_camera_view(self, px, py, pz, fx, fy, fz, ux, uy, uz):
        """Set camera to a specific view"""
        if not self.viewer:
            return
        
        camera = self.viewer.renderer.GetActiveCamera()
        camera.SetPosition(px, py, pz)
        camera.SetFocalPoint(fx, fy, fz)
        camera.SetViewUp(ux, uy, uz)
        self.viewer.renderer.ResetCameraClippingRange()
        self.viewer.vtk_widget.GetRenderWindow().Render()
        
        self.statusBar().showMessage(f"📷 Camera view updated")

    def set_default_view(self):
        """Save current camera position as default"""
        if not self.viewer:
            return
        
        camera = self.viewer.renderer.GetActiveCamera()
        default_data = {
            'focal_point': list(camera.GetFocalPoint()),
            'position': list(camera.GetPosition()),
            'view_up': list(camera.GetViewUp()),
            'clipping_range': list(camera.GetClippingRange()),
        }
        self.settings.setValue("default_camera_view", default_data)
        self.statusBar().showMessage("⭐ Default camera view saved")

    def restore_default_view(self):
        """Restore the default camera view"""
        if not self.viewer:
            return
        
        default_data = self.settings.value("default_camera_view")
        if not default_data:
            self.statusBar().showMessage("⚠ No default view saved")
            return
        
        camera = self.viewer.renderer.GetActiveCamera()
        camera.SetFocalPoint(*default_data['focal_point'])
        camera.SetPosition(*default_data['position'])
        camera.SetViewUp(*default_data['view_up'])
        camera.SetClippingRange(*default_data['clipping_range'])
        self.viewer.renderer.ResetCameraClippingRange()
        self.viewer.vtk_widget.GetRenderWindow().Render()
        
        self.statusBar().showMessage("↩️ Restored default view")

    def open_camera_preset_dialog(self):
        """Open the camera preset management dialog"""
        if not self.viewer:
            return
        
        if self.camera_preset_dialog is None:
            self.camera_preset_dialog = CameraPresetDialog(self.viewer, self)
        self.camera_preset_dialog.show()
        self.camera_preset_dialog.raise_()
        self.camera_preset_dialog.activateWindow()
        
        # Update the preset menu when dialog is closed
        def on_close():
            self.update_preset_menu()
        self.camera_preset_dialog.accepted.connect(on_close)
        self.camera_preset_dialog.rejected.connect(on_close)

    def toggle_debug_overlay(self):
        """Toggle the debug overlay visibility"""
        if self.viewer and hasattr(self.viewer, 'debug_frame'):
            self.viewer.debug_frame.setVisible(
                not self.viewer.debug_frame.isVisible()
            )

    def create_control_panel(self):
        """Create the left control panel"""
        panel = QWidget()
        panel.setStyleSheet("background-color: #0a0a1a;")
        panel.setFixedWidth(420)
        
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(15)

        # Title
        title = QLabel("MOTION PLANNING")
        title.setStyleSheet("""
            color: #6a6aae;
            font-size: 18px;
            font-weight: bold;
            letter-spacing: 2px;
            padding: 10px 0;
            border-bottom: 2px solid #2a2a4a;
        """)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        # Speed control
        speed_widget = QWidget()
        speed_layout = QHBoxLayout(speed_widget)
        speed_layout.setContentsMargins(5, 10, 5, 10)
        
        speed_label = QLabel("Speed:")
        speed_label.setStyleSheet("color: #a0a0c0; font-size: 12px;")
        speed_layout.addWidget(speed_label)
        
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 10)
        self.speed_slider.setValue(5)
        self.speed_slider.setTickInterval(1)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 4px;
                background: #2a2a4a;
                border-radius: 2px;
            }
            QSlider::handle:horizontal {
                background: #6a6aae;
                width: 16px;
                height: 16px;
                margin: -6px 0;
                border-radius: 8px;
            }
            QSlider::sub-page:horizontal {
                background: #6a6aae;
                border-radius: 2px;
            }
        """)
        self.speed_slider.valueChanged.connect(self.update_speed)
        speed_layout.addWidget(self.speed_slider)
        
        self.speed_label = QLabel("0.5x")
        self.speed_label.setStyleSheet("color: #6a6aae; font-weight: bold; font-size: 12px; min-width: 40px;")
        speed_layout.addWidget(self.speed_label)
        layout.addWidget(speed_widget)

        # Tabs
        tabs = QTabWidget()
        tabs.setStyleSheet("""
            QTabWidget::pane {
                background-color: #0f0f22;
                border: 1px solid #2a2a4a;
                border-radius: 4px;
            }
            QTabBar::tab {
                background-color: #141425;
                color: #a0a0c0;
                padding: 6px 15px;
                margin: 0 2px;
                border: 1px solid #2a2a4a;
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background-color: #1a1a3a;
                color: #ffffff;
                border-bottom: 2px solid #6a6aae;
            }
        """)
        
        joint_tab = self.create_joint_tab()
        cart_tab = self.create_cartesian_tab()
        tabs.addTab(joint_tab, "Joints")
        tabs.addTab(cart_tab, "Cartesian")
        layout.addWidget(tabs)

        # Quick camera view buttons
        camera_quick = QWidget()
        camera_quick_layout = QHBoxLayout(camera_quick)
        camera_quick_layout.setContentsMargins(0, 5, 0, 5)
        
        camera_label = QLabel("📷")
        camera_label.setStyleSheet("color: #6a6aae; font-size: 14px;")
        camera_quick_layout.addWidget(camera_label)
        
        quick_views = [
            ("Top", (0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)),
            ("Front", (0.0, -1.5, 0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 1.0)),
            ("Side", (1.5, 0.0, 0.5, 0.0, 0.0, 0.2, 0.0, 1.0, 0.0)),
            ("Iso", (1.5, -1.5, 1.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.0)),
        ]
        
        for label, params in quick_views:
            btn = QPushButton(label)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #1a1a3a;
                    color: #a0a0c0;
                    border: 1px solid #2a2a4a;
                    border-radius: 4px;
                    padding: 4px 8px;
                    font-size: 10px;
                }
                QPushButton:hover {
                    background-color: #2a2a4a;
                }
            """)
            btn.clicked.connect(lambda checked, p=params: self.set_camera_view(*p))
            camera_quick_layout.addWidget(btn)
        
        camera_quick_layout.addStretch()
        layout.addWidget(camera_quick)

        # E-stop
        estop = QPushButton("⚠ EMERGENCY STOP")
        estop.setStyleSheet("""
            QPushButton {
                background-color: #4a1a1a;
                color: #ff4444;
                border: 2px solid #ff4444;
                border-radius: 8px;
                padding: 12px;
                font-size: 16px;
                font-weight: bold;
                letter-spacing: 2px;
            }
            QPushButton:hover {
                background-color: #6a1a1a;
                border-color: #ff6666;
            }
            QPushButton:pressed {
                background-color: #2a0a0a;
            }
        """)
        estop.clicked.connect(self.emergency_stop)
        layout.addWidget(estop)

        # Status indicator
        status_widget = QWidget()
        status_layout = QHBoxLayout(status_widget)
        status_layout.setContentsMargins(5, 10, 5, 10)
        
        status_dot = QLabel("●")
        status_dot.setStyleSheet("color: #4aee6a; font-size: 12px;")
        status_layout.addWidget(status_dot)
        
        self.status_label = QLabel("Initializing...")
        self.status_label.setStyleSheet("color: #8080a0; font-size: 11px;")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()
        layout.addWidget(status_widget)

        # Version
        version = QLabel("Nextup Cobot HMI v1.0")
        version.setStyleSheet("color: #404060; font-size: 10px;")
        version.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(version)
        layout.addStretch()

        return panel

    def create_joint_tab(self):
        """Create the joint control tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 15, 10, 15)
        layout.setSpacing(8)

        header = QLabel("JOINT POSITIONS")
        header.setStyleSheet("color: #6a6aae; font-weight: bold; font-size: 12px;")
        layout.addWidget(header)

        info = QLabel("Hover + or - to preview, press and hold to move joint")
        info.setStyleSheet("color: #404060; font-size: 10px; font-style: italic;")
        layout.addWidget(info)

        # Use display labels but store with actual joint names
        joint_display_labels = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        for idx, (display_label, joint_name) in enumerate(zip(joint_display_labels, joint_names)):
            ctrl = JointControlWidget(display_label, idx)
            # Store the actual joint name in the widget
            ctrl.actual_joint_name = joint_name
            ctrl.hover_enter.connect(self.on_joint_hover_enter)
            ctrl.hover_leave.connect(self.on_joint_hover_leave)
            ctrl.joint_velocity.connect(self.on_joint_velocity)
            ctrl.joint_stop.connect(self.on_joint_stop)
            # Store by actual joint name
            self.joint_controls[joint_name] = ctrl
            layout.addWidget(ctrl)

        layout.addStretch()
        return tab

    def create_cartesian_tab(self):
        """Create the Cartesian control tab"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setContentsMargins(10, 15, 10, 15)
        layout.setSpacing(8)

        header = QLabel("CARTESIAN SPACE")
        header.setStyleSheet("color: #6a6aae; font-weight: bold; font-size: 12px;")
        layout.addWidget(header)

        info = QLabel("Hover + or - to plan motion" + (" (MoveIt)" if MOVEIT_AVAILABLE else " (disabled)"))
        info.setStyleSheet("color: #404060; font-size: 10px; font-style: italic;")
        layout.addWidget(info)

        # Translation
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

        # Rotation
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
        """Right-side panel: toggles for the hover preview modes.

        - Ghost Preview: translucent robot body showing the planned pose
          (opacity-based -- unchecking fades it to opacity 0 rather than
          tearing it down, so re-checking it mid-hover is instant).
        - Arrow Indicator: a single arrow/rotation-arc positioned from the
          SAME planned FK data, showing only the joint/axis actually about
          to move.

        Both default off; either, both, or neither can be on at once.
        """
        panel = QWidget()
        panel.setStyleSheet("background-color: #0a0a1a;")
        panel.setFixedWidth(220)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(12)

        title = QLabel("HOVER PREVIEW")
        title.setStyleSheet("""
            color: #6a6aae;
            font-size: 14px;
            font-weight: bold;
            letter-spacing: 1px;
            padding: 10px 0;
            border-bottom: 2px solid #2a2a4a;
        """)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)

        checkbox_style = """
            QCheckBox {
                color: #c0c0e0;
                font-size: 12px;
                spacing: 8px;
                padding: 4px 0;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
                border: 2px solid #3a3a5e;
                border-radius: 3px;
                background-color: #141425;
            }
            QCheckBox::indicator:checked {
                background-color: #6a6aae;
                border-color: #8a8ace;
            }
        """

        self.ghost_checkbox = QCheckBox("👻 Ghost Preview")
        self.ghost_checkbox.setStyleSheet(checkbox_style)
        self.ghost_checkbox.setChecked(False)
        self.ghost_checkbox.toggled.connect(self.on_ghost_toggle)
        layout.addWidget(self.ghost_checkbox)

        ghost_hint = QLabel("Translucent robot at the planned pose")
        ghost_hint.setStyleSheet("color: #404060; font-size: 10px; font-style: italic; padding-left: 24px;")
        ghost_hint.setWordWrap(True)
        layout.addWidget(ghost_hint)

        self.arrow_checkbox = QCheckBox("➡ Arrow Indicator")
        self.arrow_checkbox.setStyleSheet(checkbox_style)
        self.arrow_checkbox.setChecked(False)
        self.arrow_checkbox.toggled.connect(self.on_arrow_toggle)
        layout.addWidget(self.arrow_checkbox)

        arrow_hint = QLabel("Arrow at the joint/axis in motion; a rotation arc for Roll/Pitch/Yaw")
        arrow_hint.setStyleSheet("color: #404060; font-size: 10px; font-style: italic; padding-left: 24px;")
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
        """Initialize ROS2 worker"""
        self.ros_worker = RosWorker()
        self.ros_worker.node_initialized.connect(self.on_node_ready)
        self.ros_worker.start()
        self.status_label.setText("Connecting to ROS...")

    def on_node_ready(self, node):
        """Called when ROS node is ready"""
        # Setup servo publisher
        self.servo = ServoController(node)
        self.status_label.setText("Connected - Servo Active")

        # Setup MoveIt planner if available
        if MOVEIT_AVAILABLE:
            try:
                self.planner = MoveItPlanner(node, group='robot_manipulator')
                # Always wire up the ghost-preview signals once the planner
                # object exists, even if self.planner.available is False.
                # `available` only reflects whether the IK/FK services (used
                # by Cartesian hover) came up in time -- the joint-offset
                # preview doesn't touch those services at all, so gating the
                # connection on `available` used to silently break joint
                # ghost previews any time IK/FK were slow or down.
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

        # Setup TF in viewer
        self.viewer.setup_tf(node)
        print("✓ ROS2 initialized")
        
        # Update camera preset menu now that viewer is ready
        self.update_preset_menu()

    # Ghost-preview joint offset -- how far (rad) to nudge the hovered
    # joint when building the interpolated ghost trajectory. Kept modest,
    # same spirit as the Cartesian rotation_offset default.
    _JOINT_PREVIEW_OFFSET = 0.3  # rad

    # How long the pointer must sit still on a jog button before we bother
    # asking MoveIt for a plan. Below this, a hover-enter just arms the
    # timer; it does not touch the planner or the viewer at all.
    _HOVER_DEBOUNCE_MS = 120

    def _fire_pending_hover_plan(self):
        """Debounce timer fired - the pointer settled, so actually plan."""
        if self._pending_hover_plan is not None:
            self._pending_hover_plan()
            self._pending_hover_plan = None

    def on_joint_hover_enter(self, display_name, direction):
        """Handle hover entering joint button"""
        # Convert display name to actual joint name if needed
        joint_name = self.display_to_joint.get(display_name, display_name)

        # Kick off a ghost/arrow preview for this joint, same pipeline the
        # Cartesian buttons use (planner -> trajectory_ready ->
        # viewer.show_motion_preview). This only needs /joint_states, not
        # the IK/FK services, so it works even when self.planner.available
        # is False. Skip the planning round-trip entirely if neither
        # preview mode is on -- nothing would be shown anyway.
        preview_wanted = self.viewer and (self.viewer.ghost_enabled or self.viewer.arrow_enabled)
        if self.planner and preview_wanted:
            self.current_hover_axis = joint_name
            self.current_hover_direction = direction
            offset = self._JOINT_PREVIEW_OFFSET
            if direction == 'minus':
                offset = -offset

            # Don't plan immediately - arm the debounce timer. If the
            # pointer moves to another button (or off) before it fires,
            # on_joint_hover_leave() below cancels it and nothing was
            # ever sent to the FK/IK services.
            self._pending_hover_plan = lambda: self.planner.plan_joint_offset(joint_name, offset)
            self._hover_debounce_timer.start(self._HOVER_DEBOUNCE_MS)

        direction_text = '+' if direction == 'plus' else '-'
        if preview_wanted:
            self.statusBar().showMessage(f"Joint {display_name} - {direction_text}")
        else:
            self.statusBar().showMessage(f"Joint {display_name} - {direction_text} (enable Ghost/Arrow preview to see it)")

    def on_joint_hover_leave(self, display_name):
        """Handle hover leaving joint button"""
        self._hover_debounce_timer.stop()
        self._pending_hover_plan = None
        if self.viewer:
            self.viewer.clear_motion_preview()
        self.current_hover_axis = None
        self.current_hover_direction = None
        self.statusBar().showMessage("Ready")

    def on_joint_velocity(self, display_name, velocity):
        """
        Handle a joint jog button being pressed - start continuous motion.
        
        Now sends commands directly to the servo node using JointJog format.
        """
        if not self.servo:
            return
        
        # Get actual joint name from display name
        joint_name = self.display_to_joint.get(display_name, display_name)
        
        if joint_name not in self.joint_controls:
            print(f"⚠ Unknown joint: {joint_name}")
            return
        
        # Use the servo controller to send direct JointJog commands
        # Velocity is in rad/s - the servo node handles ramping
        self.servo.publish_joint_velocity_by_index(
            self.joint_controls[joint_name].joint_index,
            velocity
        )
        
        # Also send UI command for legacy compatibility (for emergency stop fallback)
        idx = self.joint_controls[joint_name].joint_index
        axis_code = str(idx + 1)
        sign = '+' if velocity > 0 else '-'
        self.servo.publish_ui_command(sign, 'j', axis_code)
        
        # The servo node only ramps up and only emits one output per
        # received message, so we must keep re-sending the SAME command while
        # held.
        if joint_name in self.active_motions:
            self.active_motions[joint_name].stop()
            self.active_motions[joint_name].deleteLater()
        
        # KEEP FAST for real robot responsiveness - 50ms
        timer = QTimer()
        timer.setInterval(50)  # FAST - 50ms for real robot
        timer.timeout.connect(lambda: self.servo.publish_joint_velocity_by_index(
            self.joint_controls[joint_name].joint_index,
            velocity
        ))
        timer.start()
        self.active_motions[joint_name] = timer

    def on_joint_stop(self, display_name):
        """Handle joint stop"""
        # Get actual joint name from display name
        joint_name = self.display_to_joint.get(display_name, display_name)
        
        if joint_name in self.active_motions:
            self.active_motions[joint_name].stop()
            self.active_motions[joint_name].deleteLater()
            del self.active_motions[joint_name]
        
        if self.servo and joint_name in self.joint_controls:
            idx = self.joint_controls[joint_name].joint_index
            # Send zero velocity via JointJog
            self.servo.publish_joint_velocity_by_index(idx, 0.0)
            # Also send UI command for legacy compatibility
            axis_code = str(idx + 1)
            self.servo.publish_ui_command('0', 'j', axis_code)

    def on_cartesian_hover_enter(self, axis, direction):
        """Handle hover entering Cartesian button - triggers planning"""
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
            
        # Determine offset
        offset = 0.1 if axis in ['x', 'y', 'z'] else 0.2  # 10cm or 0.2 rad
        if direction == 'minus':
            offset = -offset
            
        # Debounce, same as joint hover - only actually plan once the
        # pointer settles on this button.
        self._pending_hover_plan = lambda: self.planner.plan_cartesian_offset(axis, offset)
        self._hover_debounce_timer.start(self._HOVER_DEBOUNCE_MS)
        direction_text = '+' if direction == 'plus' else '-'
        self.statusBar().showMessage(f"Planning {axis} {direction_text} offset...")

    def on_cartesian_hover_leave(self, axis):
        """Handle hover leaving Cartesian button"""
        self._hover_debounce_timer.stop()
        self._pending_hover_plan = None
        self.current_hover_axis = None
        self.current_hover_direction = None
        if self.viewer:
            self.viewer.clear_motion_preview()
        self.statusBar().showMessage("Ready")

    # axis name -> the single-char code the legacy ui_command_node expects
    _CARTESIAN_AXIS_CODE = {
        'x': 'x', 'y': 'y', 'z': 'z',
        'roll': 'r', 'pitch': 'p', 'yaw': 'w',
    }

    def on_cartesian_velocity(self, axis, direction_sign):
        """
        Handle a Cartesian jog button being pressed - start continuous motion.
        
        Now sends commands directly to the servo node using TwistStamped format.
        """
        if not self.servo:
            return
        if axis not in self._CARTESIAN_AXIS_CODE:
            print(f"⚠ Unknown Cartesian axis: {axis}")
            return

        # Map axis to linear/angular components
        linear = [0.0, 0.0, 0.0]
        angular = [0.0, 0.0, 0.0]
        
        # Use the speed values from the slider
        speed = direction_sign * self.cartesian_linear_speed  # m/s
        ang_speed = direction_sign * self.cartesian_angular_speed  # rad/s
        
        if axis == 'x':
            linear[0] = speed
        elif axis == 'y':
            linear[1] = speed
        elif axis == 'z':
            linear[2] = speed
        elif axis == 'roll':
            angular[0] = ang_speed
        elif axis == 'pitch':
            angular[1] = ang_speed
        elif axis == 'yaw':
            angular[2] = ang_speed
        
        # Send direct twist command to servo node
        self.servo.publish_twist(tuple(linear), tuple(angular))
        
        # Also send UI command for legacy compatibility
        axis_code = self._CARTESIAN_AXIS_CODE[axis]
        sign = '+' if direction_sign > 0 else '-'
        self.servo.publish_ui_command(sign, 'c', axis_code)
        
        # The servo node only ramps up and only emits one output per
        # received message, so we must keep re-sending the SAME command while
        # held.
        key = f'cart_{axis}'
        if key in self.active_motions:
            self.active_motions[key].stop()
            self.active_motions[key].deleteLater()

        # KEEP FAST for real robot responsiveness - 50ms
        timer = QTimer()
        timer.setInterval(50)  # FAST - 50ms for real robot
        timer.timeout.connect(lambda: self.servo.publish_twist(tuple(linear), tuple(angular)))
        timer.start()
        self.active_motions[key] = timer

        direction_text = '+' if direction_sign > 0 else '-'
        self.statusBar().showMessage(f"Jogging {axis} {direction_text}")

    def on_cartesian_stop(self, axis):
        """Handle a Cartesian jog button being released - stop motion."""
        key = f'cart_{axis}'
        if key in self.active_motions:
            self.active_motions[key].stop()
            self.active_motions[key].deleteLater()
            del self.active_motions[key]

        if self.servo and axis in self._CARTESIAN_AXIS_CODE:
            # Send zero twist to stop
            self.servo.publish_twist((0,0,0), (0,0,0))
            # Send UI command for legacy compatibility
            axis_code = self._CARTESIAN_AXIS_CODE[axis]
            self.servo.publish_ui_command('0', 'c', axis_code)

        self.statusBar().showMessage("Ready")

    def on_trajectory_ready(self, display_trajectory, axis, direction):
        """Called when a trajectory planning request succeeds"""
        if self.current_hover_axis == axis and self.current_hover_direction == direction:
            if self.viewer:
                # axis is either a Cartesian axis code (x/y/z/roll/pitch/yaw)
                # or a joint name (joint1..joint6) -- infer which, since
                # MoveItPlanner reuses the same (trajectory, axis, direction)
                # signal shape for both.
                kind = 'cartesian' if axis in self._CARTESIAN_AXIS_CODE else 'joint'
                self.viewer.show_motion_preview(kind, axis, direction, display_trajectory)

    def on_planning_done(self, success, message):
        """Handle planning completion"""
        if success:
            self.statusBar().showMessage(f"✓ {message}")
        else:
            self.statusBar().showMessage(f"✗ {message}")
            # A failed plan never reaches on_trajectory_ready, so without
            # this the preview from whatever hover last SUCCEEDED just
            # stays on screen — looking exactly like the failed request
            # silently reused old/cached data, when really nothing new
            # ever arrived.
            if self.viewer:
                self.viewer.clear_motion_preview()

    def on_planning_progress(self, progress):
        """Handle planning progress update"""
        pass  # Could update a progress bar

    def update_speed(self, value):
        """Update speed multiplier"""
        speed = value / 10.0
        self.speed_label.setText(f"{speed:.1f}x")
        # Update joint velocity - the servo node handles ramping, we just set the target
        for ctrl in self.joint_controls.values():
            ctrl.velocity = 0.5 * speed  # Max joint speed in rad/s
        self.cartesian_linear_speed = 0.05 * speed  # m/s
        self.cartesian_angular_speed = 0.3 * speed  # rad/s

    def update_joint_display(self, joint_names, positions):
        """Update joint value displays"""
        for name, pos in zip(joint_names, positions):
            if name in self.joint_controls:
                self.joint_controls[name].update_value(pos)

    def reset_camera(self):
        """Reset 3D camera view to default position"""
        if self.viewer:
            self.viewer.reset_camera()
            self.statusBar().showMessage("Camera reset to default position")

    def emergency_stop(self):
        """Emergency stop"""
        # Stop all joint AND cartesian jog timers (active_motions holds both,
        # keyed by joint name or 'cart_<axis>')
        for timer in self.active_motions.values():
            timer.stop()
            timer.deleteLater()
        self.active_motions.clear()
        
        # Send zero velocity to all joints and stop twist via servo controller
        if self.servo:
            self.servo.publish_stop_all()
        
        self.statusBar().showMessage("⚠ EMERGENCY STOP")
        QMessageBox.warning(self, "Emergency Stop", "All joints have been stopped!")

    def show_about(self):
        """Show about dialog"""
        QMessageBox.about(self, "About Nextup Cobot HMI",
            "<h2>Nextup Cobot HMI</h2>"
            "<p>Version 1.0</p>"
            "<p>Robot control with MoveIt planning</p>"
            "<p>© Nextup Robotics</p>"
            "<br>"
            "<b>Camera Controls:</b><br>"
            "• R: Reset camera<br>"
            "• Ctrl+D: Toggle debug overlay<br>"
            "• Quick view buttons below control panel<br>"
            "• Save/load camera presets from View menu"
        )

    def closeEvent(self, event):
        """Handle close event"""
        self.emergency_stop()
        if self.ros_worker:
            self.ros_worker.stop()
            self.ros_worker.wait()
        event.accept()