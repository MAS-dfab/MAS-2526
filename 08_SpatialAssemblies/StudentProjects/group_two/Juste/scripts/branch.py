# branch.py
# r: compas>=2.14.1

import math
from compas.geometry import Line, Vector

from stick import Stick


class BranchingModule:
    """
    Simple branching L-system around a single root stick.

    - Each child grows from the previous stick ("parent").
    - Child is attached on a selected face of the parent box.
    - Child axis is a blend of parent tangent and face normal,
      controlled by a designer angle (stick_angle, in degrees).
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
            f = parent.frame

            # axis position along parent stick (offset01)
            t_param = max(0.0, min(1.0, self.offset01))
            axis_pt = parent.axis.point_at(t_param)

            # parent face normals mapped by index
            face_normals = {
                0: f.yaxis.unitized(),       # +Y
                1: f.zaxis.unitized(),       # +Z
                2: (-f.yaxis).unitized(),    # -Y
                3: (-f.zaxis).unitized()     # -Z
            }

            n = face_normals[int(face_index) % 4]

            # Parent box half-size in direction of n
            if abs(n.dot(f.yaxis)) > 0.9:
                parent_half = self.width * 0.5
            else:
                parent_half = self.depth * 0.5

            # Child box half-size in direction of n
            child_half = parent_half  # same width/depth

            # 3D offset: parent far face → child near face
            attach_pt = axis_pt + n * (parent_half + child_half)

            # Build direction blended between tangent + normal
            tangent = f.xaxis.unitized()
            theta = math.radians(float(stick_angle))
            d = n * math.cos(theta) + tangent * math.sin(theta)
            if d.length < 1e-6:
                d = tangent
            d.unitize()

            # Construct child axis so the near face sits exactly at attach_pt
            half_len = 0.5 * self.stick_length
            start = attach_pt - d * half_len
            end   = attach_pt + d * half_len
            axis  = Line(start, end)

            return Stick(axis, self.stick_length, self.width, self.depth)


    # ------------------------------------------------------------------ #
    # public: one step / multi-step growth                               #
    # ------------------------------------------------------------------ #

    def grow_once(self, face_index=0, stick_angle=0.0):

        """
        Grow one new child from the **last** stick in the chain.

        This is what RootFrames.grow_branching() calls in a loop,
        with face_index / stick_angle driven by your L-system rules.
        """
        parent = self.sticks[-1]
        child = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        """
        Convenience wrapper to grow multiple generations in one call.
        (Currently not used by RootFrames, but handy for testing.)
        """
        steps = max(1, int(steps))
        for _ in range(steps):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)
