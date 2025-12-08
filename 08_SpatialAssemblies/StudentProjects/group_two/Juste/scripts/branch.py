# branch.py
# L-system branching module with family-locked faces.

import math
from compas.geometry import Line, Vector, Frame

from stick_fixed import Stick


class BranchingModule(object):
    """
    Simple branching L-system around a single root stick.

    Face indices (local box model):

        0 -> +X (unused)
        1 -> -X (unused)
        2 -> +Y
        3 -> -Y
        4 -> +Z
        5 -> -Z

    Family rules (OPTION B with your refinement):

      - 'Y' family:
            branches ONLY on faces 2 and 3 (±Y)
      - 'Z' family:
            branches ONLY on faces 4 and 5 (±Z)

      - faces 0,1 (±X) are NEVER used.

    All descendants of a root stay in the same family.
    """

    def __init__(
        self,
        root_stick,
        stick_length=None,
        width=None,
        depth=None,
        offset01=0.5,
        family="Y",
    ):
        """
        Parameters
        ----------
        root_stick : Stick
            First stick of the chain (generation 0).
        stick_length : float, optional
        width, depth : float, optional
        offset01 : float in [0, 1]
            Parameter along the parent axis where the branch root sits.
        family : 'Y' or 'Z'
            Constrains allowed faces for branching.
        """
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.width = width or Stick.DEFAULT_SIZE
        self.depth = depth or Stick.DEFAULT_SIZE
        self.offset01 = float(offset01)
        self.family = family if family in ("Y", "Z") else "Y"

        if self.family == "Y":
            self.allowed_faces = (2, 3)
        else:
            self.allowed_faces = (4, 5)

    # ------------------------------------------------------------------ #
    # internal: face helpers                                             #
    # ------------------------------------------------------------------ #

    def _face_normal_and_halfthick(self, frame, face_index):
        """
        Return (normal, half_thickness) for a given face index.
        Uses local frame axes:

            0 -> +X
            1 -> -X
            2 -> +Y
            3 -> -Y
            4 -> +Z
            5 -> -Z
        """
        fi = int(face_index) % 6

        if fi == 0:
            n = frame.xaxis.unitized()
            half = 0.5 * self.stick_length
        elif fi == 1:
            n = (-frame.xaxis).unitized()
            half = 0.5 * self.stick_length
        elif fi == 2:
            n = frame.yaxis.unitized()
            half = 0.5 * self.width
        elif fi == 3:
            n = (-frame.yaxis).unitized()
            half = 0.5 * self.width
        elif fi == 4:
            n = frame.zaxis.unitized()
            half = 0.5 * self.depth
        else:  # fi == 5
            n = (-frame.zaxis).unitized()
            half = 0.5 * self.depth

        return n, half

    def _clamp_face_to_family(self, face_index):
        """
        Enforce family rule:
          - 'Y' => {2,3}
          - 'Z' => {4,5}
        """
        fi = int(face_index) % 6
        if self.family == "Y":
            return 2 if fi in (0, 1, 2, 4) else 3
        else:  # 'Z'
            return 4 if fi in (0, 1, 4, 2) else 5

    def _build_child_from_face(self, parent, face_index, stick_angle):
        """
        Construct a single child Stick from a parent Stick.

        - Only faces 2/3 or 4/5 are used (family-locked).
        - Child direction blends parent tangent and face normal.
        - Offsets use width/depth so boxes meet at faces.
        """
        fi = self._clamp_face_to_family(face_index)
        f = parent.frame

        # 1) position along parent axis
        t_param = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t_param)

        # 2) parent face normal & thickness
        n_parent, half_parent = self._face_normal_and_halfthick(f, fi)

        # child uses same cross-section thickness in that direction
        n_child, half_child = self._face_normal_and_halfthick(f, fi)

        # from axis → parent far face → child near face
        face_center = axis_pt + n_parent * half_parent
        child_center = face_center + n_child * half_child

        # 3) parent tangent (local x-axis)
        tangent = f.xaxis.unitized()

        # blend tangent & face normal
        theta = math.radians(float(stick_angle))
        d = tangent * math.cos(theta) + n_parent * math.sin(theta)
        if d.length < 1e-6:
            d = tangent
        d.unitize()

        # 4) build child axis centered at child_center
        half_len = 0.5 * self.stick_length
        start = child_center - d * half_len
        end = child_center + d * half_len
        axis = Line(start, end)

        # 5) build child frame: x = d, y as stable perp, z = x × y
        up_hint = f.yaxis
        if abs(up_hint.dot(d)) > 0.9:
            up_hint = f.zaxis
        zaxis = d.cross(up_hint)
        if zaxis.length < 1e-6:
            zaxis = Vector(0, 0, 1)
        zaxis.unitize()
        yaxis = zaxis.cross(d)
        if yaxis.length < 1e-6:
            yaxis = Vector(0, 1, 0)
        yaxis.unitize()

        child_frame = Frame(child_center, d, yaxis)

        child = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=child_frame,
            generation=parent.generation + 1,
            kind="branch",
            family=self.family,
        )
        return child

    # ------------------------------------------------------------------ #
    # public: grow                                                        #
    # ------------------------------------------------------------------ #

    def grow_once(self, face_index=2, stick_angle=0.0):
        """
        Grow one new child from the last stick in the chain.
        """
        parent = self.sticks[-1]
        child = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=2, stick_angle=0.0):
        steps = max(1, int(steps))
        for _ in range(steps):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)
