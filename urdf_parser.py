import os
import xml.etree.ElementTree as ET
import math

class LinkInfo:
    def __init__(self):
        self.name = ""
        self.parent = ""
        self.child = ""
        self.visual_mesh = ""
        self.origin_xyz = [0.0, 0.0, 0.0]
        self.origin_rpy = [0.0, 0.0, 0.0]
        self.joint_axis = [0.0, 0.0, 1.0]
        self.joint_type = "revolute"
        self.joint_name = ""


class URDFParser:
    def __init__(self, urdf_path=None):
        self.urdf_path = urdf_path
        self.links = {}
        self.joint_transforms = {}
        self.link_meshes = {}
        self.kinematic_chain = {}
        self.joint_axes = {}
        self.joint_chain = {}
        self.joint_names = []
        self.joint_origin_positions = {}
        self.link_to_joint = {}

        if urdf_path and os.path.exists(urdf_path):
            self.parse_urdf()
        else:
            print(f"URDF not found at {urdf_path}, using fallback")
            self._setup_fallback()

    def parse_urdf(self):
        try:
            tree = ET.parse(self.urdf_path)
            root = tree.getroot()
            for link in root.findall('link'):
                link_name = link.get('name')
                li = LinkInfo()
                li.name = link_name
                visual = link.find('visual')
                if visual is not None:
                    origin = visual.find('origin')
                    if origin is not None:
                        xyz = origin.get('xyz', '0 0 0').split()
                        rpy = origin.get('rpy', '0 0 0').split()
                        li.origin_xyz = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
                        li.origin_rpy = [float(rpy[0]), float(rpy[1]), float(rpy[2])]
                    geometry = visual.find('geometry')
                    if geometry is not None:
                        mesh = geometry.find('mesh')
                        if mesh is not None:
                            filename = mesh.get('filename')
                            if filename:
                                mesh_file = os.path.basename(filename)
                                li.visual_mesh = mesh_file
                                self.link_meshes[link_name] = mesh_file
                self.links[link_name] = li

            for joint in root.findall('joint'):
                joint_name = joint.get('name')
                joint_type = joint.get('type')
                parent = joint.find('parent')
                child = joint.find('child')
                if parent is None or child is None:
                    continue
                parent_link = parent.get('link')
                child_link = child.get('link')
                self.kinematic_chain[child_link] = parent_link
                self.joint_chain[joint_name] = (parent_link, child_link)
                self.joint_names.append(joint_name)
                self.link_to_joint[child_link] = joint_name

                origin = joint.find('origin')
                if origin is not None:
                    xyz = origin.get('xyz', '0 0 0').split()
                    rpy = origin.get('rpy', '0 0 0').split()
                    x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
                    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
                    self.joint_transforms[joint_name] = {
                        'position': (x, y, z),
                        'rotation': (roll, pitch, yaw),
                        'type': joint_type,
                        'parent': parent_link,
                        'child': child_link
                    }
                    self.joint_origin_positions[joint_name] = (x, y, z)

                axis = joint.find('axis')
                if axis is not None:
                    xyz = axis.get('xyz', '0 0 1').split()
                    axis_vec = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
                    self.joint_axes[joint_name] = axis_vec
                    if child_link in self.links:
                        self.links[child_link].joint_axis = axis_vec
                        self.links[child_link].joint_name = joint_name
                        self.links[child_link].joint_type = joint_type

            print(f"✓ URDF loaded: {len(self.links)} links, {len(self.joint_transforms)} joints")
        except Exception as e:
            print(f"Error parsing URDF: {e}")
            self._setup_fallback()

    def _setup_fallback(self):
        # Fallback transforms
        fallback = {
            'joint1': {'position': (0,0,0.23), 'parent':'base_link','child':'Link1'},
            'joint2': {'position': (0,-0.125,0.145), 'parent':'Link1','child':'Link2'},
            'joint3': {'position': (0,0.45,0), 'parent':'Link2','child':'Link3'},
            'joint4': {'position': (0,0.35,0), 'parent':'Link3','child':'Link4'},
            'joint5': {'position': (0.112,0,0.079), 'parent':'Link4','child':'Link5'},
            'joint6': {'position': (0,0.0775,0.07), 'parent':'Link5','child':'Link6'},
        }
        self.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
        # Setup basic link info
        for joint_name, data in fallback.items():
            parent = data.get('parent', '')
            child = data.get('child', '')
            pos = data.get('position', (0,0,0))
            self.joint_origin_positions[joint_name] = pos
            self.link_to_joint[child] = joint_name
            if parent and child:
                self.kinematic_chain[child] = parent
                self.joint_chain[joint_name] = (parent, child)
                if child not in self.links:
                    li = LinkInfo()
                    li.name = child
                    li.parent = parent
                    li.joint_name = joint_name
                    li.joint_type = 'revolute'
                    li.joint_axis = [0,0,1]
                    self.links[child] = li
                if 'base_link' not in self.links:
                    li = LinkInfo()
                    li.name = 'base_link'
                    li.origin_xyz = [0.0,0.0,0.0]
                    self.links['base_link'] = li

    def get_joint_chain(self, joint_name):
        return self.joint_chain.get(joint_name, (None, None))

    def get_joint_origin(self, joint_name):
        return self.joint_origin_positions.get(joint_name, (0,0,0))

    def get_joint_axis(self, joint_name):
        return self.joint_axes.get(joint_name, [0,0,1])

    def get_link_for_joint(self, joint_name):
        chain = self.joint_chain.get(joint_name, (None, None))
        return chain[1] if chain else None

    def get_visual_origin(self, link_name):
        if link_name in self.links:
            li = self.links[link_name]
            return li.origin_xyz, li.origin_rpy
        return [0,0,0], [0,0,0]