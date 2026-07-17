# joint_control_widget.py - Updated velocity scaling
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QSlider
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
from styled_button import StyledButton


class JointControlWidget(QWidget):
    hover_enter = pyqtSignal(str, str)   # display_name, direction
    hover_leave = pyqtSignal(str)        # display_name
    joint_velocity = pyqtSignal(str, float)  # display_name, velocity
    joint_stop = pyqtSignal(str)         # display_name

    def __init__(self, display_name, joint_index, parent=None):
        super().__init__(parent)
        self.display_name = display_name  # 'J1', 'J2', etc.
        self.joint_index = joint_index
        self.actual_joint_name = None  # Will be set by parent
        self.velocity = 0.5  # rad/s default - INCREASED from 0.2
        self.multiplier = 1.0
        self._last_value = 0.0  # For tracking value changes

        layout = QHBoxLayout()
        self.name_label = QLabel(display_name)
        self.name_label.setFixedWidth(30)
        
        self.minus_btn = StyledButton("−")
        self.minus_btn.setProperty("direction", "minus")
        self.value_label = QLabel("0.00")
        self.value_label.setFixedWidth(70)
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        self.plus_btn = StyledButton("+")
        self.plus_btn.setProperty("direction", "plus")

        self.minus_btn.pressed.connect(lambda: self.start_motion(-1))
        self.minus_btn.released.connect(self.stop_motion)
        self.plus_btn.pressed.connect(lambda: self.start_motion(1))
        self.plus_btn.released.connect(self.stop_motion)

        # Velocity slider
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(1, 100)
        self.slider.setValue(50)  # Default mid-range
        self.slider.valueChanged.connect(self._update_multiplier)

        layout.addWidget(self.name_label)
        layout.addWidget(self.minus_btn)
        layout.addWidget(self.value_label)
        layout.addWidget(self.plus_btn)
        layout.addWidget(self.slider)
        self.setLayout(layout)

        self.minus_btn.installEventFilter(self)
        self.plus_btn.installEventFilter(self)

    def _update_multiplier(self, value):
        """Update velocity multiplier from slider"""
        self.multiplier = value / 50.0  # 0.02 to 2.0 range
        # Update tooltip to show current speed
        current_speed = self.velocity * self.multiplier
        self.slider.setToolTip(f"Speed: {current_speed:.2f} rad/s")

    def eventFilter(self, obj, event):
        if event.type() == event.Type.Enter:
            if obj == self.minus_btn:
                self.hover_enter.emit(self.display_name, "minus")
            elif obj == self.plus_btn:
                self.hover_enter.emit(self.display_name, "plus")
        elif event.type() == event.Type.Leave:
            self.hover_leave.emit(self.display_name)
        return super().eventFilter(obj, event)

    def start_motion(self, direction):
        """Start joint motion in given direction"""
        # Use actual joint name if set, otherwise fallback to display_name
        name = self.actual_joint_name if self.actual_joint_name else self.display_name
        vel = direction * self.velocity * self.multiplier
        print(f"🔧 Joint {name} velocity: {vel:.3f} rad/s (direction: {direction})")
        self.joint_velocity.emit(name, vel)
        
        # Visual feedback
        if direction > 0:
            self.plus_btn.setStyleSheet("""
                QPushButton {
                    background-color: #4a7a4a;
                    border-color: #6aaa6a;
                    border: 2px solid #6aaa6a;
                    border-radius: 20px;
                    font-size: 18px;
                    font-weight: bold;
                    color: #ffffff;
                }
            """)
            self.minus_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2a2a3e;
                    border-color: #3a3a5e;
                    border: 2px solid #3a3a5e;
                    border-radius: 20px;
                    font-size: 18px;
                    font-weight: bold;
                    color: #ffffff;
                }
            """)
        else:
            self.minus_btn.setStyleSheet("""
                QPushButton {
                    background-color: #7a4a4a;
                    border-color: #aa6a6a;
                    border: 2px solid #aa6a6a;
                    border-radius: 20px;
                    font-size: 18px;
                    font-weight: bold;
                    color: #ffffff;
                }
            """)
            self.plus_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2a2a3e;
                    border-color: #3a3a5e;
                    border: 2px solid #3a3a5e;
                    border-radius: 20px;
                    font-size: 18px;
                    font-weight: bold;
                    color: #ffffff;
                }
            """)

    def stop_motion(self):
        """Stop joint motion"""
        name = self.actual_joint_name if self.actual_joint_name else self.display_name
        print(f"⏹ Joint {name} STOP")
        self.joint_stop.emit(name)
        self.minus_btn.setStyleSheet("")
        self.plus_btn.setStyleSheet("")

    def update_value(self, value):
        """Update displayed joint value - only if value changed significantly"""
        # Only update if value changed significantly (CPU optimization)
        if abs(value - self._last_value) < 0.001:
            return
        self._last_value = value
        
        self.value_label.setText(f"{value:.2f}")
        if abs(value) > 0.01:
            self.value_label.setStyleSheet("""
                color: #ff6b6b;
                background-color: #1a1a2e;
                border: 1px solid #ff6b6b;
                border-radius: 4px;
                padding: 4px;
                font-size: 14px;
                font-weight: bold;
                min-width: 70px;
            """)
        else:
            self.value_label.setStyleSheet("""
                color: #ffffff;
                background-color: #1a1a2e;
                border: 1px solid #3a3a5e;
                border-radius: 4px;
                padding: 4px;
                font-size: 14px;
                font-weight: bold;
                min-width: 70px;
            """)