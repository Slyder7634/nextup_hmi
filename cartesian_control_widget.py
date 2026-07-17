from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel
from PyQt6.QtCore import Qt, pyqtSignal
from styled_button import StyledButton
from theme import Palette, mono_font


class CartesianControlWidget(QWidget):
    # Hover triggers planning (ghost preview)
    hover_enter = pyqtSignal(str, str)   # axis_name, direction
    hover_leave = pyqtSignal(str)
    # Press/hold triggers actual servo motion; release stops it
    cartesian_command = pyqtSignal(str, float)  # axis_name, direction_sign (+1.0 / -1.0)
    cartesian_stop = pyqtSignal(str)            # axis_name

    _ACTIVE_POS_QSS = f"""
        QPushButton {{
            background-color: {Palette.MOTION_POS};
            border: 2px solid {Palette.MOTION_POS_BORDER};
            border-radius: 21px;
        }}
    """
    _ACTIVE_NEG_QSS = f"""
        QPushButton {{
            background-color: {Palette.MOTION_NEG};
            border: 2px solid {Palette.MOTION_NEG_BORDER};
            border-radius: 21px;
        }}
    """

    def __init__(self, axis_name, display_label, parent=None):
        super().__init__(parent)
        self.axis_name = axis_name
        self.display_label = display_label

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(8)

        self.name_label = QLabel(display_label)
        self.name_label.setFixedWidth(44)
        self.name_label.setStyleSheet(f"color: {Palette.ACCENT}; font-weight: 700; font-size: 12px;")

        self.minus_btn = StyledButton("\u2212")
        self._idle_minus_qss = self.minus_btn.styleSheet()

        self.value_label = QLabel("0.000")
        self.value_label.setFixedWidth(60)
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value_label.setFont(mono_font(11))
        self.value_label.setStyleSheet(f"color: {Palette.TEXT_MUTED};")

        self.plus_btn = StyledButton("+")
        self._idle_plus_qss = self.plus_btn.styleSheet()

        self.minus_btn.pressed.connect(lambda: self._on_press(-1))
        self.minus_btn.released.connect(self._on_release)
        self.plus_btn.pressed.connect(lambda: self._on_press(1))
        self.plus_btn.released.connect(self._on_release)

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
                self.hover_enter.emit(self.axis_name, "minus")
            elif obj == self.plus_btn:
                self.hover_enter.emit(self.axis_name, "plus")
        elif event.type() == event.Type.Leave:
            self.hover_leave.emit(self.axis_name)
        return super().eventFilter(obj, event)

    def _on_press(self, direction):
        """Button pressed - start jogging this axis in this direction."""
        self.cartesian_command.emit(self.axis_name, float(direction))
        if direction > 0:
            self.plus_btn.setStyleSheet(self._ACTIVE_POS_QSS)
            self.minus_btn.setStyleSheet(self._idle_minus_qss)
        else:
            self.minus_btn.setStyleSheet(self._ACTIVE_NEG_QSS)
            self.plus_btn.setStyleSheet(self._idle_plus_qss)

    def _on_release(self):
        """Button released - stop jogging this axis."""
        self.cartesian_stop.emit(self.axis_name)
        self.minus_btn.setStyleSheet(self._idle_minus_qss)
        self.plus_btn.setStyleSheet(self._idle_plus_qss)

    def update_value(self, value):
        self.value_label.setText(f"{value:.3f}")