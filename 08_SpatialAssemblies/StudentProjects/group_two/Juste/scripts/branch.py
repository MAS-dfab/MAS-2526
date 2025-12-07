# branch.py
# r: compas>=2.14.1

import math
from compas.geometry import Line, Vector

from stick_fixed import Stick


class BranchingModule(object):
    """
    Simple branching L-system around a single root stick.

    Behavior: child direction is a blend of parent tangent (x-axis)
    and the chosen face normal, controlled by stick_angle in degrees.
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = float(stick_length or Stick.DEFAULT_LEN)
        self.width = float(width or Stick.DEFAULT_SIZE)
        self.depth = float(depth or Stick.DEFAULT_SIZE)
        self.offset01 = float(offset01)

    # --------------------------------------------------------------
    # internal: build single child from parent face
    # --------------------------------------------------------------
    def _build_child_from_face(self, parent, face_index, stick_angle):
        f = parent.frame

        # where along parent axis the branch attaches
        t_param = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t_param)

        # choose face normal & thickness in that direction
        fi = int(face_index) % 4
        if fi == 0:   # +Y
            n = f.yaxis.unitized()
            parent_half = self.width * 0.5
        elif fi == 2:  # -Y
            n = (-f.yaxis).unitized()
            parent_half = self.width * 0.5
        elif fi == 1:  # +Z
            n = f.zaxis.unitized()
            parent_half = self.depth * 0.5
        else:          # -Z
            n = (-f.zaxis).unitized()
            parent_half = self.depth * 0.5

        child_half = parent_half

        face_center = axis_pt + n * parent_half
        child_center = face_center + n * child_half

        tangent = f.xaxis.unitized()
        theta = math.radians(float(stick_angle))
        d = tangent * math.cos(theta) + n * math.sin(theta)

        if d.length < 1e-6:
            d = tangent
        d.unitize()

        half_len = 0.5 * self.stick_length
        start = child_center - d * half_len
        end = child_center + d * half_len
        axis = Line(start, end)

        child = Stick(
            axis=axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=f,
        )
        return child

    # --------------------------------------------------------------
    # public: growth
    # --------------------------------------------------------------
    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]
        child = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        steps = max(1, int(steps))
        for _ in range(steps):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)
