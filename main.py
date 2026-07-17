#!/usr/bin/env python3
import sys
from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QPalette, QColor
from hmi_window import RobotHMI

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    # Dark palette
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(10,10,26))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(192,192,224))
    palette.setColor(QPalette.ColorRole.Base, QColor(20,20,40))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(26,26,46))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(10,10,26))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(192,192,224))
    palette.setColor(QPalette.ColorRole.Text, QColor(192,192,224))
    palette.setColor(QPalette.ColorRole.Button, QColor(20,20,40))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(192,192,224))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(60,60,140))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(255,255,255))
    app.setPalette(palette)

    window = RobotHMI()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()