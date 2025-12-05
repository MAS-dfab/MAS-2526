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
        """
        Construct a single child Stick from a parent Stick.

        face_index : 0..3
            0 -> +Y face
            1 -> +Z face
            2 -> -Y face
            3 -> -Z face
        stick_angle : float (degrees)
            Angle between the face normal and the parent tangent.
        """
        f = parent.frame          # parent local frame
        t_param = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t_param)

        # choose face normal and half-thickness in that direction
        fi = int(face_index) % 4
        if fi == 0:            # +Y
            n = f.yaxis.unitized()
            half = self.width * 0.5
        elif fi == 2:          # -Y
            n = (-f.yaxis).unitized()
            half = self.width * 0.5
        elif fi == 1:          # +Z
            n = f.zaxis.unitized()
            half = self.depth * 0.5
        else:                  # -Z
            n = (-f.zaxis).unitized()
            half = self.depth * 0.5

        # parent outer face centre
        parent_face_center = axis_pt + n * half
        # shift again by half the thickness so the child's **near face**
        # coincides with the parent's **far face**
        child_center = parent_face_center + n * half

        # parent tangent (local x)
        tangent = f.xaxis.unitized()

        # blend normal & tangent according to designer angle
        theta = math.radians(float(stick_angle))
        d = n * math.cos(theta) + tangent * math.sin(theta)

        # if degenerate, fall back to tangent
        if d.length < 1e-6:
            d = tangent
        d.unitize()

        # build child axis
        half_len = 0.5 * self.stick_length
        start = child_center - d * half_len
        end   = child_center + d * half_len
        axis  = Line(start, end)

        # Stick will compute its own frame from the axis
        child = Stick(axis, length=self.stick_length,
                      width=self.width, depth=self.depth)
        return child

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
