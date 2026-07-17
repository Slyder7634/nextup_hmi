# joint_control_widget.py
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel
from PyQt6.QtCore import Qt, pyqtSignal
from styled_button import StyledButton
from theme import Palette, mono_font


class JointControlWidget(QWidget):
    hover_enter = pyqtSignal(str, str)       # display_name, direction
    hover_leave = pyqtSignal(str)            # display_name
    joint_velocity = pyqtSignal(str, float)  # display_name, velocity
    joint_stop = pyqtSignal(str)             # display_name

    _IDLE_BTN_QSS = ""  # StyledButton's own default stylesheet
    _ACTIVE_POS_QSS = f"""
        QPushButton {{
            background-color: {Palette.MOTION_POS};
            border: 2px solid {Palette.MOTION_POS_BORDER};
            border-radius: 21px;
            font-size: 18px;
            font-weight: bold;
            color: #ffffff;
        }}
    """
    _ACTIVE_NEG_QSS = f"""
        QPushButton {{
            background-color: {Palette.MOTION_NEG};
            border: 2px solid {Palette.MOTION_NEG_BORDER};
            border-radius: 21px;
            font-size: 18px;
            font-weight: bold;
            color: #ffffff;
        }}
    """

    def __init__(self, display_name, joint_index, parent=None):
        super().__init__(parent)
        self.display_name = display_name
        self.joint_index = joint_index
        self.actual_joint_name = None  # set by parent
        self.velocity = 0.5  # rad/s, scaled by the panel's global Speed slider
        self._last_value = 0.0

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(8)

        self.name_label = QLabel(display_name)
        self.name_label.setFixedWidth(28)
        self.name_label.setStyleSheet(f"color: {Palette.ACCENT}; font-weight: 700; font-size: 13px;")

        self.minus_btn = StyledButton("\u2212")
        self._idle_minus_qss = self.minus_btn.styleSheet()

        self.value_label = QLabel("0.00")
        self.value_label.setFixedWidth(68)
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value_label.setFont(mono_font(12, bold=True))
        self._style_value_label(active=False)

        self.plus_btn = StyledButton("+")
        self._idle_plus_qss = self.plus_btn.styleSheet()

        self.minus_btn.pressed.connect(lambda: self.start_motion(-1))
        self.minus_btn.released.connect(self.stop_motion)
        self.plus_btn.pressed.connect(lambda: self.start_motion(1))
        self.plus_btn.released.connect(self.stop_motion)

        layout.addWidget(self.name_label)
        layout.addWidget(self.minus_btn)
        layout.addWidget(self.value_label)
        layout.addWidget(self.plus_btn)
        layout.addStretch()
        self.setLayout(layout)

        self.minus_btn.installEventFilter(self)
        self.plus_btn.installEventFilter(self)

    def eventFilter(self, obj, event):
        if event.type() == event.Type.Enter:
            if obj == self.minus_btn:
                self.hover_enter.emit(self.display_name, "minus")
            elif obj == self.plus_btn:
                self.hover_enter.emit(self.display_name, "plus")
        elif event.type() == event.Type.Leave:
            self.hover_leave.emit(self.display_name)
        return super().eventFilter(obj, event)

    def _joint_name(self):
        return self.actual_joint_name if self.actual_joint_name else self.display_name

    def start_motion(self, direction):
        """Start joint motion in given direction."""
        name = self._joint_name()
        vel = direction * self.velocity
        self.joint_velocity.emit(name, vel)

        if direction > 0:
            self.plus_btn.setStyleSheet(self._ACTIVE_POS_QSS)
            self.minus_btn.setStyleSheet(self._idle_minus_qss)
        else:
            self.minus_btn.setStyleSheet(self._ACTIVE_NEG_QSS)
            self.plus_btn.setStyleSheet(self._idle_plus_qss)

    def stop_motion(self):
        """Stop joint motion."""
        self.joint_stop.emit(self._joint_name())
        self.minus_btn.setStyleSheet(self._idle_minus_qss)
        self.plus_btn.setStyleSheet(self._idle_plus_qss)

    def _style_value_label(self, active):
        if active:
            self.value_label.setStyleSheet(f"""
                color: {Palette.DANGER_HOVER};
                background-color: {Palette.SURFACE};
                border: 1px solid {Palette.DANGER_HOVER};
                border-radius: 4px;
                padding: 4px;
            """)
        else:
            self.value_label.setStyleSheet(f"""
                color: {Palette.TEXT};
                background-color: {Palette.SURFACE};
                border: 1px solid {Palette.BORDER};
                border-radius: 4px;
                padding: 4px;
            """)

    def update_value(self, value):
        """Update displayed joint value - only if it changed significantly."""
        if abs(value - self._last_value) < 0.001:
            return
        self._last_value = value
        self.value_label.setText(f"{value:.2f}")
        self._style_value_label(active=abs(value) > 0.01)