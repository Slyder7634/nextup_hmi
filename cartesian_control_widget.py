from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel
from PyQt6.QtCore import Qt, pyqtSignal
from styled_button import StyledButton

class CartesianControlWidget(QWidget):
    # Hover triggers planning (ghost preview)
    hover_enter = pyqtSignal(str, str)   # axis_name, direction
    hover_leave = pyqtSignal(str)
    # Press/hold triggers actual servo motion; release stops it
    cartesian_command = pyqtSignal(str, float)  # axis_name, direction_sign (+1.0 / -1.0)
    cartesian_stop = pyqtSignal(str)            # axis_name

    def __init__(self, axis_name, display_label, parent=None):
        super().__init__(parent)
        self.axis_name = axis_name
        self.display_label = display_label

        layout = QHBoxLayout()
        self.name_label = QLabel(display_label)
        self.minus_btn = StyledButton("−")
        self.value_label = QLabel("0.00")
        self.plus_btn = StyledButton("+")

        self.minus_btn.pressed.connect(lambda: self._on_press(-1))
        self.minus_btn.released.connect(self._on_release)
        self.plus_btn.pressed.connect(lambda: self._on_press(1))
        self.plus_btn.released.connect(self._on_release)

        layout.addWidget(self.name_label)
        layout.addWidget(self.minus_btn)
        layout.addWidget(self.value_label)
        layout.addWidget(self.plus_btn)
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
        # Visual feedback, same idea as JointControlWidget
        if direction > 0:
            self.plus_btn.setStyleSheet("background-color: #4a7a4a; border-color: #6aaa6a;")
            self.minus_btn.setStyleSheet("")
        else:
            self.minus_btn.setStyleSheet("background-color: #7a4a4a; border-color: #aa6a6a;")
            self.plus_btn.setStyleSheet("")

    def _on_release(self):
        """Button released - stop jogging this axis."""
        self.cartesian_stop.emit(self.axis_name)
        self.minus_btn.setStyleSheet("")
        self.plus_btn.setStyleSheet("")

    def update_value(self, value):
        self.value_label.setText(f"{value:.3f}")