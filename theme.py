"""
theme.py - Single source of truth for the Nextup Cobot HMI look & feel.

Why this exists
----------------
Before this file, color values and QSS fragments were copy-pasted across
hmi_window.py, joint_control_widget.py, cartesian_control_widget.py and
styled_button.py (the same ~15 hex codes appeared 100+ times). Changing
the accent color meant hunting through five files. Now every widget pulls
from `Palette` and the QSS builders below, so the whole app can be
re-themed by editing one file.

Visual identity
----------------
"Reactor console": a near-black instrument panel with a single vivid
cyan accent for normal operation, amber for caution, red for danger,
and green for "connected/live". Numeric readouts use a monospace font
so joint values line up and feel like real telemetry.
"""

from PyQt6.QtGui import QFont

# --------------------------------------------------------------------------
# Palette
# --------------------------------------------------------------------------

class Palette:
    # Surfaces (dark theme)
    BG = "#090c12"            # window background
    SURFACE = "#0d121b"       # panels
    SURFACE_ALT = "#111826"   # menu bars, status bar
    CARD = "#141b28"          # cards / tab pages / list backgrounds
    CARD_RAISED = "#182233"   # buttons, inputs
    BORDER = "#232d3d"        # default border
    BORDER_SOFT = "#1a2230"   # subtle divider

    # Text
    TEXT = "#dbe4f0"          # primary text
    TEXT_MUTED = "#8492a6"    # secondary text
    TEXT_DIM = "#4a5568"      # tertiary / hints

    # Accents
    ACCENT = "#22d3ee"        # cyan - primary interactive accent
    ACCENT_HOVER = "#67e8f9"  # cyan hover / bright
    ACCENT_DIM = "#155e75"    # cyan pressed / dim border

    WARNING = "#ffb020"       # amber - caution
    DANGER = "#ff3b5c"        # red - emergency / stop
    DANGER_HOVER = "#ff6b85"
    DANGER_DIM = "#3a0f18"
    SUCCESS = "#2dd97a"       # green - connected / positive motion
    INFO = "#8b5cf6"          # violet - secondary highlight (used sparingly)

    # Motion feedback (jog buttons)
    MOTION_POS = "#1f7a4d"    # '+' direction pressed
    MOTION_POS_BORDER = "#2dd97a"
    MOTION_NEG = "#8a2a3a"    # '-' direction pressed
    MOTION_NEG_BORDER = "#ff6b85"

    # Light theme (kept minimal, mirrors dark structure)
    class Light:
        BG = "#f2f4f8"
        SURFACE = "#e9ecf3"
        CARD = "#ffffff"
        CARD_RAISED = "#eef1f7"
        BORDER = "#d3d9e6"
        TEXT = "#1c2333"
        TEXT_MUTED = "#5b6479"
        ACCENT = "#0891b2"
        ACCENT_HOVER = "#0e7490"
        DANGER = "#e0304f"
        SUCCESS = "#1a9d5c"


MONO_FONT_FAMILY = "'JetBrains Mono', 'Cascadia Code', 'Consolas', monospace"
UI_FONT_FAMILY = "'Segoe UI', 'Inter', sans-serif"


def mono_font(size=13, bold=False):
    f = QFont("Consolas")
    f.setStyleHint(QFont.StyleHint.Monospace)
    f.setPointSize(size)
    f.setBold(bold)
    return f


# --------------------------------------------------------------------------
# Reusable QSS fragments
# --------------------------------------------------------------------------

def button_qss(bg=None, fg=None, border=None, hover_bg=None, hover_border=None,
                pressed_bg=None, radius=8, padding="8px 14px", font_size=13,
                bold=False, letter_spacing=None):
    """Build a QPushButton stylesheet from a handful of semantic colors."""
    bg = bg or Palette.CARD_RAISED
    fg = fg or Palette.TEXT
    border = border or Palette.BORDER
    hover_bg = hover_bg or Palette.BORDER
    hover_border = hover_border or Palette.ACCENT_DIM
    pressed_bg = pressed_bg or Palette.ACCENT_DIM
    weight = "font-weight: 600;" if bold else ""
    spacing = f"letter-spacing: {letter_spacing}px;" if letter_spacing else ""
    return f"""
        QPushButton {{
            background-color: {bg};
            color: {fg};
            border: 1px solid {border};
            border-radius: {radius}px;
            padding: {padding};
            font-size: {font_size}px;
            {weight}
            {spacing}
        }}
        QPushButton:hover {{
            background-color: {hover_bg};
            border-color: {hover_border};
        }}
        QPushButton:pressed {{
            background-color: {pressed_bg};
        }}
        QPushButton:disabled {{
            color: {Palette.TEXT_DIM};
            border-color: {Palette.BORDER_SOFT};
        }}
    """


def card_frame_qss(radius=10):
    return f"""
        QFrame#Card {{
            background-color: {Palette.CARD};
            border: 1px solid {Palette.BORDER};
            border-radius: {radius}px;
        }}
    """


def section_title_qss(color=None, size=14):
    color = color or Palette.ACCENT
    return f"""
        color: {color};
        font-size: {size}px;
        font-weight: 700;
        letter-spacing: 2px;
        padding: 8px 0;
        border-bottom: 1px solid {Palette.BORDER};
    """


def checkbox_qss():
    return f"""
        QCheckBox {{
            color: {Palette.TEXT};
            font-size: 12px;
            spacing: 8px;
            padding: 2px 0;
        }}
        QCheckBox::indicator {{
            width: 15px;
            height: 15px;
            border: 2px solid {Palette.BORDER};
            border-radius: 4px;
            background-color: {Palette.SURFACE};
        }}
        QCheckBox::indicator:hover {{
            border-color: {Palette.ACCENT_DIM};
        }}
        QCheckBox::indicator:checked {{
            background-color: {Palette.ACCENT};
            border-color: {Palette.ACCENT_HOVER};
        }}
    """


def slider_qss(accent=None):
    accent = accent or Palette.ACCENT
    return f"""
        QSlider::groove:horizontal {{
            height: 4px;
            background: {Palette.BORDER};
            border-radius: 2px;
        }}
        QSlider::handle:horizontal {{
            background: {accent};
            width: 16px;
            height: 16px;
            margin: -6px 0;
            border-radius: 8px;
        }}
        QSlider::sub-page:horizontal {{
            background: {accent};
            border-radius: 2px;
        }}
    """


def app_qss(theme="Dark"):
    """The global application stylesheet (menu bar, tabs, inputs, lists...)."""
    P = Palette if theme != "Light" else Palette.Light
    accent_border_selected = getattr(P, "ACCENT", Palette.ACCENT)
    return f"""
        QMainWindow {{
            background-color: {P.BG};
        }}
        QMenuBar {{
            background-color: {P.SURFACE};
            color: {P.TEXT_MUTED};
            border-bottom: 1px solid {P.BORDER};
            padding: 2px;
        }}
        QMenuBar::item {{
            padding: 4px 10px;
            border-radius: 4px;
            background: transparent;
        }}
        QMenuBar::item:selected {{
            background-color: {P.BORDER};
            color: {P.TEXT};
        }}
        QMenu {{
            background-color: {P.SURFACE};
            color: {P.TEXT_MUTED};
            border: 1px solid {P.BORDER};
        }}
        QMenu::item {{
            padding: 5px 20px;
        }}
        QMenu::item:selected {{
            background-color: {P.BORDER};
            color: {P.TEXT};
        }}
        QTabWidget::pane {{
            background-color: {P.CARD};
            border: 1px solid {P.BORDER};
            border-radius: 6px;
        }}
        QTabBar::tab {{
            background-color: {P.SURFACE};
            color: {P.TEXT_MUTED};
            padding: 7px 18px;
            margin: 0 2px;
            border: 1px solid {P.BORDER};
            border-bottom: none;
            border-top-left-radius: 6px;
            border-top-right-radius: 6px;
        }}
        QTabBar::tab:selected {{
            background-color: {P.CARD};
            color: {P.TEXT};
            border-bottom: 2px solid {accent_border_selected};
        }}
        QTabBar::tab:hover {{
            color: {P.TEXT};
        }}
        QLabel {{
            color: {P.TEXT};
        }}
        QGroupBox {{
            color: {getattr(P, "ACCENT", Palette.ACCENT)};
            font-weight: 600;
            border: 1px solid {P.BORDER};
            border-radius: 8px;
            margin-top: 10px;
            padding-top: 12px;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
        }}
        QStatusBar {{
            background-color: {P.SURFACE};
            color: {P.TEXT_MUTED};
            border-top: 1px solid {P.BORDER};
        }}
        QPushButton {{
            background-color: {getattr(P, "CARD_RAISED", P.CARD)};
            color: {P.TEXT};
            border: 1px solid {P.BORDER};
            border-radius: 6px;
            padding: 5px 10px;
        }}
        QPushButton:hover {{
            background-color: {P.BORDER};
        }}
        {slider_qss(getattr(P, "ACCENT", Palette.ACCENT))}
        {checkbox_qss() if theme != "Light" else ""}
        QLineEdit, QComboBox {{
            background-color: {P.SURFACE};
            color: {P.TEXT};
            border: 1px solid {P.BORDER};
            border-radius: 4px;
            padding: 4px 6px;
        }}
        QLineEdit:focus, QComboBox:focus {{
            border-color: {getattr(P, "ACCENT", Palette.ACCENT)};
        }}
        QTreeWidget, QListWidget {{
            background-color: {P.SURFACE};
            color: {P.TEXT};
            border: 1px solid {P.BORDER};
            border-radius: 6px;
        }}
        QTreeWidget::item:selected, QListWidget::item:selected {{
            background-color: {P.BORDER};
        }}
        QHeaderView::section {{
            background-color: {P.SURFACE};
            color: {P.TEXT_MUTED};
            padding: 3px 4px;
            border: none;
            border-bottom: 1px solid {P.BORDER};
        }}
        QScrollBar:vertical {{
            background: {P.BG};
            width: 10px;
            margin: 0;
        }}
        QScrollBar::handle:vertical {{
            background: {P.BORDER};
            border-radius: 5px;
            min-height: 24px;
        }}
        QScrollBar::handle:vertical:hover {{
            background: {getattr(P, "ACCENT_DIM", P.BORDER)};
        }}
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
            height: 0;
        }}
    """