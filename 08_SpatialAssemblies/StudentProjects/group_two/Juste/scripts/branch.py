# branch.py
# r: compas>=2.14.1

import math
from compas.geometry import Line, Vector

from stick_fixed import Stick


class BranchingModule:
    """
    Simple branching L-system around a single root stick.

    - Each child grows from the previous stick ("parent").
    - Child is attached on a selected face of the parent box.
    - Child axis is a blend of parent tangent and face normal,
      controlled by stick_angle (degrees).
    - Parent and child share the same cross-section (width/depth).
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        """
        Parameters
        ----------
        root_stick : Stick
            First stick of the chain, used as parent for generation 0.
        stick_length : float, optional
            Length of all child sticks (and root if you wish).
        width : float, optional
        depth : float, optional
        offset01 : float in [0, 1]
            Parameter along the parent **axis** where the branch root sits.
            0.0 = at parent start, 1.0 = at parent end.
        """
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.width = width or Stick.DEFAULT_SIZE
        self.depth = depth or Stick.DEFAULT_SIZE
        self.offset01 = float(offset01)

    # ------------------------------------------------------------------ #
    # internal: build one child on a parent face                         #
    # ------------------------------------------------------------------ #

    def _build_child_from_face(self, parent, face_index, stick_angle):
        """
        Construct a single child Stick from a parent Stick.

        face_index : 0..3
            0 -> +Y face
            1 -> +Z face
            2 -> -Y face
            3 -> -Z face
        stick_angle : float (degrees)
            Angle between the face normal and the parent tangent.

        Behavior:
            Child direction is a blend of parent tangent (xaxis) and
            face normal (y/z), so it "grows tangentially along the
            surface normal".
        """
        f = parent.frame

        # 1) Position along parent axis
        t_param = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t_param)

        # 2) Face normal & “half-thickness” in that direction
        fi = int(face_index) % 4
        if fi == 0:          # +Y
            n = f.yaxis.unitized()
            parent_half = self.width * 0.5
        elif fi == 2:        # -Y
            n = (-f.yaxis).unitized()
            parent_half = self.width * 0.5
        elif fi == 1:        # +Z
            n = f.zaxis.unitized()
            parent_half = self.depth * 0.5
        else:                # -Z
            n = (-f.zaxis).unitized()
            parent_half = self.depth * 0.5

        child_half = parent_half

        # 3) From axis → parent far face → child near face
        face_center = axis_pt + n * parent_half
        child_center = face_center + n * child_half

        # 4) Blend parent tangent and face normal
        tangent = f.xaxis.unitized()
        theta = math.radians(float(stick_angle))
        d = tangent * math.cos(theta) + n * math.sin(theta)

        if d.length < 1e-6:
            d = tangent
        d.unitize()

        # 5) Child axis centered at child_center
        half_len = 0.5 * self.stick_length
        start = child_center - d * half_len
        end = child_center + d * half_len
        axis = Line(start, end)

        # inherit parent frame orientation for local axes
        child = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=f
        )
        return child

    # ------------------------------------------------------------------ #
    # public: one step / multi-step growth                               #
    # ------------------------------------------------------------------ #

    def grow_once(self, face_index=0, stick_angle=0.0):
        """Grow one new child from the **last** stick in the chain."""
        parent = self.sticks[-1]
        child = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        """Convenience wrapper to grow multiple generations in one call."""
        steps = max(1, int(steps))
        for _ in range(steps):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)
