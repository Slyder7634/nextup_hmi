from PyQt6.QtWidgets import QPushButton

class StyledButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setFixedSize(40,40)
        self.setStyleSheet("""
            QPushButton {
                background-color: #2a2a3e;
                color: #ffffff;
                border: 2px solid #3a3a5e;
                border-radius: 20px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #3a3a5e;
                border-color: #5a5a8e;
            }
            QPushButton:pressed {
                background-color: #4a4a6e;
            }
        """)