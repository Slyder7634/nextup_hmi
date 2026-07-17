"""
VTK 3D Robot Viewer for Nextup Cobot HMI - Ultra Optimized
"""
import os
import math
import sys
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy, QLabel, QFrame, QGridLayout
from PyQt6.QtCore import QTimer, pyqtSignal, Qt
import rclpy
from geometry_msgs.msg import TransformStamped

try:
    import vtk
    from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
    VTK_AVAILABLE = True
except ImportError:
    VTK_AVAILABLE = False
    print("Error: VTK not installed")

try:
    import tf2_ros
    from tf2_ros import TransformException
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

try:
    from moveit_msgs.msg import RobotState
except ImportError:
    RobotState = None

from urdf_parser import URDFParser


class Robot3DViewer(QWidget):
    """VTK-based 3D robot visualization widget - Ultra Optimized"""
    
    joint_values_updated = pyqtSignal(list, list)
    
    def __init__(self, parent=None, urdf_path=None, mesh_path=None):
        super().__init__(parent)
        
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Find URDF
        if urdf_path is None:
            candidates = [
                os.path.join(script_dir, "nextupCobot.urdf.xacro"),
                os.path.join(script_dir, "nextupCobot.urdf"),
                os.path.expanduser("~/NextupRobot/src/nextup_moveit_config/urdf/nextupCobot.urdf.xacro"),
            ]
            for c in candidates:
                if os.path.exists(c):
                    urdf_path = c
                    break
        
        self.urdf_path = urdf_path
        self.mesh_path = mesh_path or os.path.join(script_dir, "meshes/")
        self.urdf_parser = URDFParser(self.urdf_path)
        
        # Real robot actors
        self.link_actors = {}
        self.link_transforms = {}
        self.link_colors = {}
        self.link_original_colors = {}
        self.link_names = []
        
        # Ghost robot actors
        self.ghost_actors = []
        self.ghost_link_actors = {}
        self.ghost_visible = False
        self.ghost_waypoints = []
        
        # Animation system for trajectory
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animate_ghost_frame)
        self.animation_frame = 0
        self.animation_speed = 250  # 4Hz - Very slow for CPU
            
        # Ghost / Arrow preview toggles
        self.ghost_enabled = False
        self.arrow_enabled = False
        self._last_preview = None

        # Motion indicator actors
        self.motion_arrow_actor = None
        self.rpy_arc_plus = None
        self.rpy_arc_minus = None
        
        # TF
        self.tf_buffer = None
        self.tf_listener = None
        self.tf_initialized = False
        
        # End-effector
        self.ee_link_name = 'link6'
        self.ee_axes_actor = None
        self.ghost_ee_axes_actor = None
        
        # Store latest pose for debug
        self.latest_ee_pose = None
        self.latest_joint_positions = {}
        
        # Lazy rendering
        self._needs_rerender = False
        self._is_visible = True
        self._frame_counter = 0
        self._last_rendered_poses = {}
        self._last_debug_state = {}
        
        # Setup VTK with lower quality
        self.setup_vtk()
        self.load_robot_meshes()
        self.setup_camera()
        self.create_motion_indicators()
        self.create_ee_axes()
        
        # Timer for TF updates
        self.update_timer = QTimer()
        self.update_timer.setInterval(70)
        self.update_timer.timeout.connect(self.update_from_tf)
        self.update_timer.start()
        
        # Debug overlay (non-critical, very slow)
        self.setup_debug_overlay()
        
        # Visibility tracking
        self.installEventFilter(self)
    
    def eventFilter(self, obj, event):
        """Track visibility changes for lazy rendering"""
        if event.type() == event.Type.Show:
            self._is_visible = True
            self._needs_rerender = True
            self.update_from_tf()
        elif event.type() == event.Type.Hide:
            self._is_visible = False
        return super().eventFilter(obj, event)
    
    def setup_debug_overlay(self):
        """Create a debug overlay - updated VERY slowly"""
        self.debug_frame = QFrame(self)
        self.debug_frame.setStyleSheet("""
            QFrame {
                background-color: rgba(10, 10, 26, 200);
                border: 1px solid #2a2a4a;
                border-radius: 6px;
                padding: 8px;
            }
            QLabel {
                color: #c0c0e0;
                font-family: monospace;
                font-size: 10px;
                background-color: transparent;
            }
            .title-label {
                color: #6a6aae;
                font-weight: bold;
                font-size: 11px;
            }
            .value-label {
                color: #8a8ace;
                font-weight: bold;
            }
            .status-ok {
                color: #4aee6a;
            }
            .status-warning {
                color: #ffaa44;
            }
            .status-error {
                color: #ff4444;
            }
            .section-label {
                color: #6a6aae;
                font-size: 9px;
                font-weight: bold;
                border-bottom: 1px solid #2a2a4a;
                padding-bottom: 2px;
            }
        """)
        self.debug_frame.setFixedWidth(350)
        self.debug_frame.move(10, 10)
        
        debug_layout = QGridLayout(self.debug_frame)
        debug_layout.setSpacing(2)
        debug_layout.setContentsMargins(8, 8, 8, 8)
        
        row = 0
        title_label = QLabel("🔧 ROBOT STATE")
        title_label.setProperty("class", "title-label")
        debug_layout.addWidget(title_label, row, 0, 1, 2)
        row += 1
        
        sep = QLabel("─" * 35)
        sep.setStyleSheet("color: #2a2a4a;")
        debug_layout.addWidget(sep, row, 0, 1, 2)
        row += 1
        
        debug_layout.addWidget(QLabel("Pos:"), row, 0)
        self.ee_pos_label = QLabel("--")
        self.ee_pos_label.setProperty("class", "value-label")
        debug_layout.addWidget(self.ee_pos_label, row, 1)
        row += 1
        
        debug_layout.addWidget(QLabel("RPY:"), row, 0)
        self.ee_rpy_label = QLabel("--")
        self.ee_rpy_label.setProperty("class", "value-label")
        debug_layout.addWidget(self.ee_rpy_label, row, 1)
        row += 1
        
        sep2 = QLabel("─" * 35)
        sep2.setStyleSheet("color: #2a2a4a;")
        debug_layout.addWidget(sep2, row, 0, 1, 2)
        row += 1
        
        self.joint_labels = {}
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        display_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        
        for i, (jname, dname) in enumerate(zip(joint_names, display_names)):
            col = i % 2
            grid_row = row + (i // 2)
            debug_layout.addWidget(QLabel(f"{dname}:"), grid_row, col * 2)
            label = QLabel("--")
            label.setProperty("class", "value-label")
            label.setFixedWidth(60)
            self.joint_labels[jname] = label
            debug_layout.addWidget(label, grid_row, col * 2 + 1)
        
        row += 3
        
        self.debug_frame.raise_()
        self.debug_frame.show()
        
        # Debug update - VERY slow (1Hz)
        self.debug_timer = QTimer()
        self.debug_timer.setInterval(1000)
        self.debug_timer.timeout.connect(self.update_debug_info)
        self.debug_timer.start()
    
    def quaternion_to_rpy(self, qx, qy, qz, qw):
        """Convert quaternion to roll, pitch, yaw"""
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def _values_unchanged(self):
        """Check if debug values have changed"""
        current_state = {
            'ee_pos': self.latest_ee_pose['position'] if self.latest_ee_pose else None,
            'ee_orient': self.latest_ee_pose['orientation'] if self.latest_ee_pose else None,
            'joints': self.latest_joint_positions.copy() if self.latest_joint_positions else {},
            'tf_init': self.tf_initialized,
        }
        
        if hasattr(self, '_last_debug_state'):
            if current_state == self._last_debug_state:
                return True
        
        self._last_debug_state = current_state
        return False
    
    def update_debug_info(self):
        """Update debug overlay - only if values changed"""
        try:
            if self._values_unchanged():
                return
            
            if self.latest_ee_pose:
                pos = self.latest_ee_pose['position']
                orient = self.latest_ee_pose['orientation']
                
                self.ee_pos_label.setText(f"({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
                self.ee_pos_label.setStyleSheet("color: #8a8ace; font-weight: bold;")
                
                roll, pitch, yaw = self.quaternion_to_rpy(orient[0], orient[1], orient[2], orient[3])
                self.ee_rpy_label.setText(f"({math.degrees(roll):.1f}°, {math.degrees(pitch):.1f}°, {math.degrees(yaw):.1f}°)")
                self.ee_rpy_label.setStyleSheet("color: #8a8ace; font-weight: bold;")
            else:
                self.ee_pos_label.setText("--")
                self.ee_rpy_label.setText("--")
            
            if self.latest_joint_positions:
                for joint_name, label in self.joint_labels.items():
                    if joint_name in self.latest_joint_positions:
                        val = self.latest_joint_positions[joint_name]
                        label.setText(f"{val:.3f}")
                        label.setStyleSheet("color: #8a8ace; font-weight: bold;")
                    else:
                        label.setText("--")
                        label.setStyleSheet("color: #666;")
            
        except Exception:
            pass
    
    def setup_vtk(self):
        """Initialize VTK renderer with lower quality settings"""
        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.vtk_widget.setSizePolicy(QSizePolicy.Policy.Expanding, 
                                     QSizePolicy.Policy.Expanding)
        
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.03, 0.03, 0.06)
        
        # Disable expensive features
        self.renderer.SetUseFXAA(False)
        self.renderer.SetUseHiddenLineRemoval(False)
        
        # Setup minimal lighting
        self.setup_lighting()
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)
        
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        self.interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        
        # Reduce render quality
        render_window = self.vtk_widget.GetRenderWindow()
        render_window.SetMultiSamples(0)
        render_window.SetSwapBuffers(True)
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.vtk_widget)
    
    def setup_lighting(self):
        """Setup minimal lighting"""
        ambient = vtk.vtkLight()
        ambient.SetColor(0.5, 0.5, 0.6)
        ambient.SetIntensity(0.5)
        self.renderer.AddLight(ambient)
        
        light = vtk.vtkLight()
        light.SetPosition(1.5, -1.0, 1.5)
        light.SetFocalPoint(0, 0.2, 0.4)
        light.SetColor(1.0, 1.0, 1.0)
        light.SetIntensity(0.8)
        light.SetLightTypeToSceneLight()
        self.renderer.AddLight(light)
    
    def setup_camera(self):
        """Setup compact camera view"""
        camera = self.renderer.GetActiveCamera()
        camera.SetFocalPoint(0.0, 0.1, 0.2)
        camera.SetPosition(0.0, -1.2, 0.6)
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCameraClippingRange()
        self.add_compact_grid()
    
    def reset_camera(self):
        """Reset camera to default compact position"""
        camera = self.renderer.GetActiveCamera()
        camera.SetFocalPoint(0.0, 0.1, 0.2)
        camera.SetPosition(0.0, -1.2, 0.6)
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCameraClippingRange()
        self._needs_rerender = True
        self._render_if_needed()
    
    def add_compact_grid(self):
        """Add a smaller, more efficient grid"""
        grid_size = 0.8
        grid_resolution = 8
        
        grid_source = vtk.vtkPlaneSource()
        grid_source.SetOrigin(-grid_size, -grid_size, 0.0)
        grid_source.SetPoint1(grid_size, -grid_size, 0.0)
        grid_source.SetPoint2(-grid_size, grid_size, 0.0)
        grid_source.SetXResolution(grid_resolution)
        grid_source.SetYResolution(grid_resolution)
        
        grid_mapper = vtk.vtkPolyDataMapper()
        grid_mapper.SetInputConnection(grid_source.GetOutputPort())
        
        grid_actor = vtk.vtkActor()
        grid_actor.SetMapper(grid_mapper)
        grid_actor.GetProperty().SetColor(0.25, 0.25, 0.35)
        grid_actor.GetProperty().SetOpacity(0.4)
        grid_actor.GetProperty().SetRepresentationToWireframe()
        
        self.renderer.AddActor(grid_actor)
        
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.15, 0.15, 0.15)
        axes.SetShaftType(vtk.vtkAxesActor.LINE_SHAFT)
        axes.SetCylinderRadius(0.02)
        axes.SetConeRadius(0.1)
        axes.AxisLabelsOff()
        self.renderer.AddActor(axes)
    
    def load_robot_meshes(self):
        """Load robot meshes with lower quality settings"""
        if self.urdf_parser.link_meshes:
            link_meshes = self.urdf_parser.link_meshes
        else:
            link_meshes = {
                'base_link': 'base_link.STL',
                'Link1': 'Link1.STL',
                'Link2': 'Link2.STL',
                'Link3': 'Link3.STL',
                'Link4': 'Link4.STL',
                'Link5': 'Link5.STL',
                'Link6': 'Link6.STL',
            }
        
        colors = {
            'base_link': (0.4, 0.4, 0.5),
            'Link1': (0.5, 0.6, 0.7),
            'Link2': (0.6, 0.7, 0.8),
            'Link3': (0.65, 0.75, 0.85),
            'Link4': (0.7, 0.8, 0.9),
            'Link5': (0.75, 0.82, 0.92),
            'Link6': (0.8, 0.85, 0.95),
        }
        
        for link_name in self.urdf_parser.link_meshes.keys():
            if link_name in ["world", "ground_link", "end"]:
                continue
            
            mesh_file = None
            for mesh_link, mesh_file_name in link_meshes.items():
                if mesh_link.lower() == link_name.lower():
                    mesh_file = mesh_file_name
                    break
            
            if mesh_file is None:
                for mesh_link, mesh_file_name in link_meshes.items():
                    if link_name.lower() in mesh_link.lower() or mesh_link.lower() in link_name.lower():
                        mesh_file = mesh_file_name
                        break
            
            if mesh_file is None:
                mesh_file = 'base_link.STL'
            
            color = colors.get(link_name, (0.7, 0.7, 0.8))
            if link_name not in colors:
                for key in colors:
                    if key.lower() == link_name.lower():
                        color = colors[key]
                        break
            
            self.link_colors[link_name] = color
            self.link_original_colors[link_name] = color
            actor = self._load_mesh(mesh_file, color)
            self.link_actors[link_name] = actor
            self.link_names.append(link_name)
            self.renderer.AddActor(actor)
            
            transform = vtk.vtkTransform()
            transform.Identity()
            self.link_transforms[link_name] = transform
            actor.SetUserTransform(transform)
        
        for link_name, actor in self.link_actors.items():
            ghost_actor = vtk.vtkActor()
            ghost_actor.SetMapper(actor.GetMapper())
            ghost_actor.GetProperty().SetColor(0.0, 0.8, 0.8)
            ghost_actor.GetProperty().SetOpacity(0.4)
            ghost_actor.GetProperty().SetSpecular(0.05)
            ghost_actor.SetVisibility(False)
            self.renderer.AddActor(ghost_actor)
            self.ghost_link_actors[link_name] = ghost_actor
    
    def _load_mesh(self, mesh_filename, color):
        """Load a mesh file with lower quality settings"""
        try:
            mesh_paths = [
                os.path.join(self.mesh_path, mesh_filename),
                os.path.join(self.mesh_path, mesh_filename.lower()),
                os.path.join(self.mesh_path, mesh_filename.upper()),
                os.path.join(self.mesh_path, mesh_filename.capitalize()),
                mesh_filename,
            ]
            
            for path in mesh_paths:
                if os.path.exists(path):
                    if path.lower().endswith('.stl'):
                        reader = vtk.vtkSTLReader()
                        reader.SetFileName(path)
                        mapper = vtk.vtkPolyDataMapper()
                        mapper.SetInputConnection(reader.GetOutputPort())
                        actor = vtk.vtkActor()
                        actor.SetMapper(mapper)
                        actor.GetProperty().SetColor(color)
                        actor.GetProperty().SetSpecular(0.1)
                        actor.GetProperty().SetSpecularPower(10)
                        actor.GetProperty().SetInterpolationToFlat()
                        return actor
                    elif path.lower().endswith('.obj'):
                        reader = vtk.vtkOBJReader()
                        reader.SetFileName(path)
                        mapper = vtk.vtkPolyDataMapper()
                        mapper.SetInputConnection(reader.GetOutputPort())
                        actor = vtk.vtkActor()
                        actor.SetMapper(mapper)
                        actor.GetProperty().SetColor(color)
                        actor.GetProperty().SetSpecular(0.1)
                        actor.GetProperty().SetSpecularPower(10)
                        actor.GetProperty().SetInterpolationToFlat()
                        return actor
            
            source = vtk.vtkCubeSource()
            source.SetXLength(0.04)
            source.SetYLength(0.04)
            source.SetZLength(0.04)
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(source.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(color)
            actor.GetProperty().SetInterpolationToFlat()
            return actor
            
        except Exception:
            source = vtk.vtkSphereSource()
            source.SetRadius(0.02)
            source.SetThetaResolution(12)
            source.SetPhiResolution(12)
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(source.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(color)
            actor.GetProperty().SetInterpolationToFlat()
            return actor
    
    def create_ee_axes(self):
        """Build end-effector coordinate triads - smaller"""
        self.ee_axes_actor = self._make_axes_actor(length=0.1, opacity=1.0)
        self.ee_axes_actor.SetVisibility(False)
        self.renderer.AddActor(self.ee_axes_actor)
        
        self.ghost_ee_axes_actor = self._make_axes_actor(length=0.1, opacity=0.55)
        self.ghost_ee_axes_actor.SetVisibility(False)
        self.renderer.AddActor(self.ghost_ee_axes_actor)
    
    def _make_axes_actor(self, length=0.1, opacity=1.0):
        """Build one solid XYZ coordinate triad - smaller and lighter"""
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(length, length, length)
        axes.SetShaftType(vtk.vtkAxesActor.LINE_SHAFT)
        axes.SetCylinderRadius(0.04)
        axes.SetConeRadius(0.25)
        axes.AxisLabelsOff()
        
        if opacity < 1.0:
            for shaft_prop, tip_prop in (
                (axes.GetXAxisShaftProperty(), axes.GetXAxisTipProperty()),
                (axes.GetYAxisShaftProperty(), axes.GetYAxisTipProperty()),
                (axes.GetZAxisShaftProperty(), axes.GetZAxisTipProperty()),
            ):
                shaft_prop.SetOpacity(opacity)
                tip_prop.SetOpacity(opacity)
        
        return axes
    
    def create_motion_indicators(self):
        """Build motion indicator actors - smaller"""
        arrow_source = vtk.vtkArrowSource()
        arrow_source.SetTipLength(0.2)
        arrow_source.SetTipRadius(0.06)
        arrow_source.SetShaftRadius(0.025)

        self.motion_arrow_actor = vtk.vtkActor()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(arrow_source.GetOutputPort())
        self.motion_arrow_actor.SetMapper(mapper)
        self.motion_arrow_actor.SetVisibility(False)
        self.renderer.AddActor(self.motion_arrow_actor)

        self.rpy_arc_plus = self._create_circular_arrow(False, 0.0, 0.0, 0.0)
        self.rpy_arc_minus = self._create_circular_arrow(True, 0.0, 0.0, 0.0)
        for arc in (self.rpy_arc_plus, self.rpy_arc_minus):
            arc.SetVisibility(False)
            self.renderer.AddActor(arc)

    def _create_circular_arrow(self, clockwise, cx, cy, cz):
        """Create a circular arrow - lower resolution"""
        points = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        
        num_segments = 20
        radius = 0.2
        
        if clockwise:
            start = 0
            end = -2 * math.pi * 0.75
            step = (end - start) / num_segments
        else:
            start = 0
            end = 2 * math.pi * 0.75
            step = (end - start) / num_segments
        
        point_ids = []
        angle = start
        for i in range(num_segments + 1):
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            pid = points.InsertNextPoint(x, y, cz)
            point_ids.append(pid)
            angle += step
        
        polyline = vtk.vtkPolyLine()
        polyline.GetPointIds().SetNumberOfIds(len(point_ids))
        for i, pid in enumerate(point_ids):
            polyline.GetPointIds().SetId(i, pid)
        lines.InsertNextCell(polyline)
        
        arc = vtk.vtkPolyData()
        arc.SetPoints(points)
        arc.SetLines(lines)
        
        if len(point_ids) >= 2:
            p1 = points.GetPoint(point_ids[-2])
            p2 = points.GetPoint(point_ids[-1])
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0.001:
                dx /= length
                dy /= length
                
                head_len = 0.04
                head_wid = 0.02
                
                head_pts = vtk.vtkPoints()
                head_pts.InsertNextPoint(p2[0], p2[1], cz)
                head_pts.InsertNextPoint(
                    p2[0] - dx*head_len + dy*head_wid,
                    p2[1] - dy*head_len - dx*head_wid,
                    cz
                )
                head_pts.InsertNextPoint(
                    p2[0] - dx*head_len - dy*head_wid,
                    p2[1] - dy*head_len + dx*head_wid,
                    cz
                )
                
                head_cells = vtk.vtkCellArray()
                head_cells.InsertNextCell(3)
                head_cells.InsertCellPoint(0)
                head_cells.InsertCellPoint(1)
                head_cells.InsertCellPoint(2)
                
                head_poly = vtk.vtkPolyData()
                head_poly.SetPoints(head_pts)
                head_poly.SetPolys(head_cells)
                
                append = vtk.vtkAppendPolyData()
                append.AddInputData(arc)
                append.AddInputData(head_poly)
                append.Update()
                combined = append.GetOutput()
            else:
                combined = arc
        else:
            combined = arc
        
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(combined)
        
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetLineWidth(1.5)
        actor.GetProperty().SetOpacity(0.0)
        
        return actor
    
    def _axis_alignment_transform(self, target_axis):
        """Build transform to align arrow with target axis"""
        tx, ty, tz = target_axis
        norm = math.sqrt(tx * tx + ty * ty + tz * tz)
        if norm < 1e-9:
            tx, ty, tz = 1.0, 0.0, 0.0
        else:
            tx, ty, tz = tx / norm, ty / norm, tz / norm

        t = vtk.vtkTransform()
        t.Identity()

        dot = tx
        cx, cy, cz = 0.0, -tz, ty
        cross_norm = math.sqrt(cx * cx + cy * cy + cz * cz)

        if cross_norm < 1e-6:
            if dot < 0:
                t.RotateWXYZ(180.0, 0.0, 0.0, 1.0)
        else:
            angle = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
            t.RotateWXYZ(angle, cx, cy, cz)

        return t

    def _rpy_alignment_transform(self, rpy_name):
        """Rotate arc for RPY axes"""
        t = vtk.vtkTransform()
        t.Identity()
        if rpy_name == 'roll':
            t.RotateY(90.0)
        elif rpy_name == 'pitch':
            t.RotateX(-90.0)
        return t

    def _place_straight_arrow(self, base_tf, local_axis, color, scale=0.1):
        """Position the reusable arrow - smaller scale"""
        align = self._axis_alignment_transform(local_axis)
        combined = vtk.vtkTransform()
        combined.Concatenate(base_tf)
        combined.Concatenate(align)
        combined.Scale(scale, scale, scale)

        self.motion_arrow_actor.SetUserTransform(combined)
        self.motion_arrow_actor.GetProperty().SetColor(*color)
        self.motion_arrow_actor.GetProperty().SetOpacity(1.0)
        self.motion_arrow_actor.SetVisibility(True)

    def _place_rotation_arc(self, ee_tf, rpy_name, direction, color):
        """Position a rotation arc"""
        align = self._rpy_alignment_transform(rpy_name)
        combined = vtk.vtkTransform()
        combined.Concatenate(ee_tf)
        combined.Concatenate(align)
        combined.Scale(0.35, 0.35, 0.35)

        arc = self.rpy_arc_plus if direction == 'plus' else self.rpy_arc_minus
        arc.SetUserTransform(combined)
        arc.GetProperty().SetColor(*color)
        arc.GetProperty().SetOpacity(1.0)
        arc.SetVisibility(True)

    def _show_motion_arrow(self, kind, name, direction, joint_positions):
        """Show arrow indicator - only if arrow enabled"""
        self.hide_motion_arrow()
        if not self.arrow_enabled:
            return

        color = (0.0, 0.8, 0.0) if direction == 'plus' else (0.8, 0.0, 0.0)

        if kind == 'joint':
            child_link = self.urdf_parser.get_link_for_joint(name)
            axis = self.urdf_parser.get_joint_axis(name)
            if not child_link:
                return
            link_tf = self.get_transform_to_link(child_link, joint_positions)
            self._place_straight_arrow(link_tf, axis, color, scale=0.1)

        elif kind == 'cartesian':
            ee_tf = self.get_transform_to_link(self.ee_link_name, joint_positions)
            if name in ('x', 'y', 'z'):
                local_axis = {'x': (1.0, 0.0, 0.0), 'y': (0.0, 1.0, 0.0), 'z': (0.0, 0.0, 1.0)}[name]
                self._place_straight_arrow(ee_tf, local_axis, color, scale=0.15)
            elif name in ('roll', 'pitch', 'yaw'):
                self._place_rotation_arc(ee_tf, name, direction, color)

        self._needs_rerender = True
        self._render_if_needed()

    def hide_motion_arrow(self):
        """Hide arrow indicator"""
        if self.motion_arrow_actor is not None:
            self.motion_arrow_actor.SetVisibility(False)
        for arc in (self.rpy_arc_plus, self.rpy_arc_minus):
            if arc is not None:
                arc.SetVisibility(False)
        self._needs_rerender = True
    
    def setup_tf(self, node):
        """Setup TF listener"""
        if ROS2_AVAILABLE and node is not None:
            try:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
                self.tf_initialized = True
                return True
            except Exception:
                return False
        return False
    
    def _should_render(self):
        """Lazy rendering - only render when needed and visible"""
        if not self._is_visible:
            self._needs_rerender = False
            return False
        
        if not self._needs_rerender:
            return False
        
        # Throttle rendering
        self._frame_counter += 1
        if self._frame_counter % 2 == 0:
            return False
        
        self._needs_rerender = False
        return True
    
    def _render_if_needed(self):
        """Render only if needed and visible"""
        if self._should_render() and self.vtk_widget:
            self.vtk_widget.GetRenderWindow().Render()
    
    def update_from_tf(self):
        """Update robot poses from TF"""
        if not self.tf_initialized or self.tf_buffer is None or not self._is_visible:
            return
        
        try:
            any_update = False
            live_transforms = self._lookup_live_link_transforms(self.link_actors.keys())
            
            for link_name, vtk_transform in live_transforms.items():
                actor = self.link_actors[link_name]
                actor.SetUserTransform(vtk_transform)
                self.link_transforms[link_name] = vtk_transform
                any_update = True
            
            if self.ee_axes_actor is not None:
                ee_transform = self._lookup_live_link_transforms(
                    [self.ee_link_name], apply_visual_offset=False
                ).get(self.ee_link_name)
                if ee_transform is not None:
                    self.ee_axes_actor.SetUserTransform(ee_transform)
                    self.ee_axes_actor.SetVisibility(True)
                    
                    pos = ee_transform.GetPosition()
                    orient = ee_transform.GetOrientationWXYZ()
                    angle = math.radians(orient[0])
                    qx = orient[1] * math.sin(angle/2)
                    qy = orient[2] * math.sin(angle/2)
                    qz = orient[3] * math.sin(angle/2)
                    qw = math.cos(angle/2)
                    self.latest_ee_pose = {
                        'position': (pos[0], pos[1], pos[2]),
                        'orientation': (qx, qy, qz, qw)
                    }
                    any_update = True
            
            if any_update:
                self._needs_rerender = True
                self._render_if_needed()
                
        except Exception:
            pass

    def _lookup_live_link_transforms(self, link_names, apply_visual_offset=True):
        """Look up TF transforms for links"""
        transforms = {}
        if not self.tf_initialized or self.tf_buffer is None:
            return transforms
        
        now = rclpy.time.Time()
        timeout = rclpy.duration.Duration(seconds=0.01)
        
        for link_name in link_names:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', link_name, now, timeout=timeout
                )
                if not transform:
                    continue
                
                vtk_transform = self._convert_tf_to_vtk(transform)
                
                if apply_visual_offset:
                    xyz, rpy = self.urdf_parser.get_visual_origin(link_name)
                    if xyz != [0,0,0] or rpy != [0,0,0]:
                        offset = vtk.vtkTransform()
                        offset.Translate(xyz[0], xyz[1], xyz[2])
                        offset.RotateZ(math.degrees(rpy[2]))
                        offset.RotateY(math.degrees(rpy[1]))
                        offset.RotateX(math.degrees(rpy[0]))
                        
                        combined = vtk.vtkTransform()
                        combined.Concatenate(vtk_transform)
                        combined.Concatenate(offset)
                        vtk_transform = combined
                
                transforms[link_name] = vtk_transform
                
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                pass
            except Exception:
                pass
        
        return transforms
    
    def _convert_tf_to_vtk(self, transform_stamped):
        """Convert ROS TransformStamped to vtkTransform"""
        vtk_transform = vtk.vtkTransform()
        vtk_transform.Identity()
        
        trans = transform_stamped.transform.translation
        vtk_transform.Translate(trans.x, trans.y, trans.z)
        
        q = transform_stamped.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm > 1e-6:
            qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        
        matrix = vtk.vtkMatrix4x4()
        matrix.Identity()
        
        xx, yy, zz = qx*qx, qy*qy, qz*qz
        xy, xz, yz = qx*qy, qx*qz, qy*qz
        wx, wy, wz = qw*qx, qw*qy, qw*qz
        
        matrix.SetElement(0, 0, 1 - 2*(yy + zz))
        matrix.SetElement(0, 1, 2*(xy - wz))
        matrix.SetElement(0, 2, 2*(xz + wy))
        
        matrix.SetElement(1, 0, 2*(xy + wz))
        matrix.SetElement(1, 1, 1 - 2*(xx + zz))
        matrix.SetElement(1, 2, 2*(yz - wx))
        
        matrix.SetElement(2, 0, 2*(xz - wy))
        matrix.SetElement(2, 1, 2*(yz + wx))
        matrix.SetElement(2, 2, 1 - 2*(xx + yy))
        
        vtk_transform.Concatenate(matrix)
        return vtk_transform
    
    def get_transform_to_link(self, link_name, joint_positions):
        """Compute FK transform for a link"""
        path = []
        curr = link_name
        while curr in self.urdf_parser.kinematic_chain and curr != 'base_link':
            parent = self.urdf_parser.kinematic_chain[curr]
            joint_name = self.urdf_parser.link_to_joint.get(curr)
            path.append((parent, joint_name, curr))
            curr = parent
        
        path.reverse()
        
        t = vtk.vtkTransform()
        t.Identity()
        
        for parent, joint_name, child in path:
            joint_data = self.urdf_parser.joint_transforms.get(joint_name)
            if not joint_data:
                continue
                
            pos = joint_data['position']
            rot = joint_data.get('rotation', (0.0, 0.0, 0.0))
            
            t.Translate(pos[0], pos[1], pos[2])
            t.RotateZ(math.degrees(rot[2]))
            t.RotateY(math.degrees(rot[1]))
            t.RotateX(math.degrees(rot[0]))
            
            joint_type = joint_data.get('type', 'revolute')
            if joint_type in ['revolute', 'continuous']:
                angle = joint_positions.get(joint_name, 0.0)
                axis = self.urdf_parser.joint_axes.get(joint_name, [0.0, 0.0, 1.0])
                t.RotateWXYZ(math.degrees(angle), axis[0], axis[1], axis[2])
        
        xyz, rpy = self.urdf_parser.get_visual_origin(link_name)
        if xyz != [0,0,0] or rpy != [0,0,0]:
            offset = vtk.vtkTransform()
            offset.Translate(xyz[0], xyz[1], xyz[2])
            offset.RotateZ(math.degrees(rpy[2]))
            offset.RotateY(math.degrees(rpy[1]))
            offset.RotateX(math.degrees(rpy[0]))
            
            combined = vtk.vtkTransform()
            combined.Concatenate(t)
            combined.Concatenate(offset)
            t = combined
        
        return t

    def _validate_joint_names(self, joint_names):
        """Validate joint names against URDF"""
        known = set(self.urdf_parser.joint_transforms.keys())
        given = set(joint_names)
        missing = given - known
        if missing:
            return False
        return True

    _GHOST_LINK_OPACITY = 0.4
    _GHOST_EE_AXES_OPACITY = 0.55
    _GHOST_PATH_OPACITY = 0.8

    def show_motion_preview(self, kind, name, direction, display_trajectory):
        """Show ghost/arrow preview - only if ghost enabled"""
        joint_positions = self._extract_start_joint_positions(display_trajectory)
        if joint_positions is None:
            return

        self._last_preview = {
            'kind': kind,
            'name': name,
            'direction': direction,
            'display_trajectory': display_trajectory,
            'joint_positions': joint_positions,
        }
        self._refresh_ghost()
        self._refresh_arrow()

    def clear_motion_preview(self):
        """Clear both preview modes"""
        self._last_preview = None
        self.clear_ghost()
        self.hide_motion_arrow()

    def set_ghost_enabled(self, enabled):
        """Enable/disable ghost preview"""
        self.ghost_enabled = enabled
        self._refresh_ghost()

    def set_arrow_enabled(self, enabled):
        """Enable/disable arrow indicator"""
        self.arrow_enabled = enabled
        self._refresh_arrow()

    def _refresh_ghost(self):
        """Refresh ghost preview - only if enabled"""
        if self._last_preview is None:
            return
        if self.ghost_enabled:
            self._build_ghost_actors(self._last_preview['display_trajectory'])
        elif self.ghost_waypoints:
            self._set_ghost_opacity(False)

    def _refresh_arrow(self):
        """Refresh arrow indicator - only if enabled"""
        if self._last_preview is None or not self.arrow_enabled:
            self.hide_motion_arrow()
            return
        p = self._last_preview
        self._show_motion_arrow(p['kind'], p['name'], p['direction'], p['joint_positions'])

    def _extract_start_joint_positions(self, display_trajectory):
        """Extract joint positions from trajectory"""
        if not display_trajectory or not display_trajectory.trajectory:
            return None
        traj = display_trajectory.trajectory[0]
        if not traj.joint_trajectory.points:
            return None
        joint_names = traj.joint_trajectory.joint_names
        first_point = traj.joint_trajectory.points[0]
        return dict(zip(joint_names, first_point.positions))

    def _set_axes_opacity(self, axes_actor, opacity):
        """Set opacity for axes actor"""
        for shaft_prop, tip_prop in (
            (axes_actor.GetXAxisShaftProperty(), axes_actor.GetXAxisTipProperty()),
            (axes_actor.GetYAxisShaftProperty(), axes_actor.GetYAxisTipProperty()),
            (axes_actor.GetZAxisShaftProperty(), axes_actor.GetZAxisTipProperty()),
        ):
            shaft_prop.SetOpacity(opacity)
            tip_prop.SetOpacity(opacity)

    def _set_ghost_opacity(self, on):
        """Fade ghost in/out"""
        op_link = self._GHOST_LINK_OPACITY if on else 0.0
        for actor in self.ghost_link_actors.values():
            actor.GetProperty().SetOpacity(op_link)

        if self.ghost_ee_axes_actor is not None:
            self._set_axes_opacity(self.ghost_ee_axes_actor, self._GHOST_EE_AXES_OPACITY if on else 0.0)

        op_path = self._GHOST_PATH_OPACITY if on else 0.0
        for actor in self.ghost_actors:
            actor.GetProperty().SetOpacity(op_path)

        self._needs_rerender = True
        self._render_if_needed()

    def _build_ghost_actors(self, display_trajectory):
        """Build ghost robot - ONLY if ghost is enabled"""
        self.clear_ghost()
        
        if not self.ghost_enabled:
            return
        
        if not display_trajectory or not display_trajectory.trajectory:
            return
        
        model_states = []
        if display_trajectory.trajectory and display_trajectory.trajectory[0].joint_trajectory.points:
            traj = display_trajectory.trajectory[0]
            joint_names = traj.joint_trajectory.joint_names
            self._validate_joint_names(joint_names)
            
            for pt in traj.joint_trajectory.points:
                state = RobotState()
                state.joint_state.name = joint_names
                state.joint_state.position = pt.positions
                state.joint_state.velocity = pt.velocities if pt.velocities else [0.0] * len(joint_names)
                model_states.append(state)
        
        if not model_states:
            return
        
        self.ghost_waypoints = model_states
        self.ghost_visible = True
        self.animation_frame = 0
        
        start_state = model_states[0]
        joint_positions = dict(zip(start_state.joint_state.name, start_state.joint_state.position))
        
        for link_name, actor in self.ghost_link_actors.items():
            vtk_transform = self.get_transform_to_link(link_name, joint_positions)
            actor.SetUserTransform(vtk_transform)
            actor.SetVisibility(True)
        
        ee_link = self.ee_link_name
        if ee_link not in self.ghost_link_actors:
            for k in self.ghost_link_actors.keys():
                if k.lower() == self.ee_link_name.lower():
                    ee_link = k
                    break
        
        if self.ghost_ee_axes_actor is not None:
            ee_start_transform = self.get_transform_to_link(ee_link, joint_positions)
            self.ghost_ee_axes_actor.SetUserTransform(ee_start_transform)
            self.ghost_ee_axes_actor.SetVisibility(True)
            
        for i, state in enumerate(model_states):
            state_joints = dict(zip(state.joint_state.name, state.joint_state.position))
            ee_tf = self.get_transform_to_link(ee_link, state_joints)
            pos = ee_tf.GetPosition()
            
            sphere = vtk.vtkSphereSource()
            sphere.SetRadius(0.008)
            sphere.SetThetaResolution(8)
            sphere.SetPhiResolution(8)
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(sphere.GetOutputPort())
            
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            
            t = i / max(1, len(model_states) - 1)
            color = (0.0, 1.0 - 0.5 * t, 0.5 + 0.5 * t)
            actor.GetProperty().SetColor(color)
            actor.SetPosition(pos)
            
            self.renderer.AddActor(actor)
            self.ghost_actors.append(actor)
        
        self._set_ghost_opacity(self.ghost_enabled)
        self._start_animation()
        self._needs_rerender = True
        self._render_if_needed()
    
    def clear_ghost(self):
        """Remove ghost robot"""
        self._stop_animation()
        for actor in self.ghost_link_actors.values():
            actor.SetVisibility(False)
        
        if self.ghost_ee_axes_actor is not None:
            self.ghost_ee_axes_actor.SetVisibility(False)
            
        for actor in self.ghost_actors:
            self.renderer.RemoveActor(actor)
        self.ghost_actors.clear()
        self.ghost_waypoints.clear()
        self.ghost_visible = False
        self.animation_frame = 0
        
        self._needs_rerender = True
        self._render_if_needed()
    
    # ========== ANIMATION METHODS ==========
    
    def _start_animation(self):
        """Start ghost animation - only if ghost enabled"""
        if not self.ghost_enabled:
            return
        self.animation_frame = 0
        self.animation_timer.start(self.animation_speed)
    
    def _stop_animation(self):
        """Stop ghost animation"""
        self.animation_timer.stop()
    
    def _animate_ghost_frame(self):
        """Animate ghost - only if ghost enabled and visible"""
        if not self.ghost_enabled or not self.ghost_visible:
            return
        
        if not self.ghost_waypoints or not self._is_visible:
            return
        
        num_waypoints = len(self.ghost_waypoints)
        if num_waypoints == 0:
            return
        
        self.animation_frame = self.animation_frame % num_waypoints
        current_state = self.ghost_waypoints[self.animation_frame]
        joint_positions = dict(zip(current_state.joint_state.name, current_state.joint_state.position))
        
        for link_name, actor in self.ghost_link_actors.items():
            vtk_transform = self.get_transform_to_link(link_name, joint_positions)
            actor.SetUserTransform(vtk_transform)
            actor.SetVisibility(True)
        
        if self.ghost_ee_axes_actor is not None:
            ee_link = self.ee_link_name
            if ee_link not in self.ghost_link_actors:
                for k in self.ghost_link_actors.keys():
                    if k.lower() == self.ee_link_name.lower():
                        ee_link = k
                        break
            
            ee_transform = self.get_transform_to_link(ee_link, joint_positions)
            self.ghost_ee_axes_actor.SetUserTransform(ee_transform)
            self.ghost_ee_axes_actor.SetVisibility(True)
        
        self.animation_frame += 1
        
        self._needs_rerender = True
        self._render_if_needed()
    
    def set_animation_speed(self, ms_per_frame):
        """Set animation speed"""
        self.animation_speed = max(10, min(500, ms_per_frame))
        if self.animation_timer.isActive():
            self.animation_timer.stop()
            self.animation_timer.start(self.animation_speed)
    
    def get_animation_speed(self):
        """Get animation speed"""
        return self.animation_speed