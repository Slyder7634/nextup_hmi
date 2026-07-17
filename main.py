#!/usr/bin/env python3
import sys
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QPalette, QColor
from hmi_window import RobotHMI
from theme import Palette as P

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    # Dark palette - mirrors theme.py so native dialogs/tooltips match the app
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(P.BG))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(P.TEXT))
    palette.setColor(QPalette.ColorRole.Base, QColor(P.SURFACE))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(P.CARD))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(P.SURFACE))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(P.TEXT))
    palette.setColor(QPalette.ColorRole.Text, QColor(P.TEXT))
    palette.setColor(QPalette.ColorRole.Button, QColor(P.CARD_RAISED))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(P.TEXT))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(P.ACCENT_DIM))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(P.TEXT))
    app.setPalette(palette)

    window = RobotHMI()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()