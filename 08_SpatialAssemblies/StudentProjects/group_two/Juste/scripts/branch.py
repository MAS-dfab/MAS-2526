# branch.py

import math
from compas.geometry import Line, Frame, Vector
from stick import Stick, stable_perp


class BranchingModule:
    """
    Face-anchored branching:
    - child sits on parent face
    - child axis = blend(parent-tangent, face-normal)
    """
    def __init__(self, root_stick, stick_length, width, depth, offset01):
        self.sticks = [root_stick]
        self.stick_length = stick_length
        self.width = width
        self.depth = depth
        self.offset01 = offset01

    # ---------------------------------------------
    # FACE INDEX → NORMAL + HALF THICKNESS
    # ---------------------------------------------
    def _face_info(self, parent_frame, face_index):
        fi = face_index % 4
        if fi == 0:
            return parent_frame.yaxis.unitized(), self.width * 0.5
        elif fi == 2:
            return (-parent_frame.yaxis).unitized(), self.width * 0.5
        elif fi == 1:
            return parent_frame.zaxis.unitized(), self.depth * 0.5
        else:
            return (-parent_frame.zaxis).unitized(), self.depth * 0.5

    # ---------------------------------------------
    # BUILD A CHILD STICK
    # ---------------------------------------------
    def build_child(self, parent, face_index, angle_deg):
        pf = parent.frame

        # position along parent axis
        t = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t)

        n, half = self._face_info(pf, face_index)

        parent_face_center = axis_pt + n * half
        child_center = parent_face_center + n * half

        tangent = pf.xaxis
        tangent_proj = tangent - n * tangent.dot(n)
        if tangent_proj.length < 1e-6:
            tangent_proj = stable_perp(n)
        tangent_proj.unitize()

        theta = math.radians(angle_deg)
        d_raw = n * math.cos(theta) + tangent_proj * math.sin(theta)
        d = d_raw - n * d_raw.dot(n)
        if d.length < 1e-6:
            d = tangent_proj
        d.unitize()

        x = d
        y = n
        z = x.cross(y).unitized()
        child_frame = Frame(child_center, x, y)

        half_len = self.stick_length * 0.5
        start = child_center - x * half_len
        end   = child_center + x * half_len
        axis = Line(start, end)

        child = Stick(axis, self.stick_length, self.width, self.depth)
        child.frame = child_frame
        return child

    def grow_once(self, face_index, angle_deg):
        parent = self.sticks[-1]
        child = self.build_child(parent, face_index, angle_deg)
        self.sticks.append(child)

    def grow_chain(self, steps, face_index, angle_deg):
        for _ in range(int(steps)):
            self.grow_once(face_index, angle_deg)
