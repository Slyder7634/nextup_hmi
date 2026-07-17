from PyQt6.QtWidgets import QPushButton
from PyQt6.QtGui import QCursor
from PyQt6.QtCore import Qt
from theme import Palette, button_qss


class StyledButton(QPushButton):
    """A round jog button (the +/- controls for joints & Cartesian axes)."""

    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setFixedSize(42, 42)
        self.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.setStyleSheet(button_qss(
            bg=Palette.CARD_RAISED,
            fg=Palette.TEXT,
            border=Palette.BORDER,
            hover_bg=Palette.BORDER,
            hover_border=Palette.ACCENT,
            pressed_bg=Palette.ACCENT_DIM,
            radius=21,
            padding="0px",
            font_size=18,
            bold=True,
        ))