# branch.py
# BranchingModule for COMPAS-based spatial L-system
# Implements:
#   - Allowed face families (Y or Z)
#   - Local alternation logic (triggered only when bridging is needed)
#   - One-sided offsets using parent.width or parent.depth
#   - Tangent + face-normal blend direction
#   - Creates children as COMPAS Box sticks

import math
import random
from compas.geometry import Line, Vector

from stick_fixed import Stick


# =====================================================================
# Allowed face mapping (we decided on this explicitly)
# =====================================================================

Y_FACES = [0, 2]   # +Y, -Y
Z_FACES = [1, 3]   # +Z, -Z

# faces 0/1/2/3 correspond to local Stick.frame axes:
#   X = tangent direction
#   Y = width direction
#   Z = depth direction


# =====================================================================
# Branching Module
# =====================================================================

class BranchingModule:
    """
    Branching is an L-system:
        - Each new child grows from the last stick in the chain.
        - Children attach on either the Y-family faces OR the Z-family faces.
        - The parent determines NEXT family unless local alternation is triggered.

    Local alternation rule:
        If rf_core detects that this parent needs bridging,
        parent.family flips: Y → Z or Z → Y.
    """

    def __init__(
        self,
        root_stick: Stick,
        stick_length=None,
        width=None,
        depth=None,
        offset01=0.5,
        initial_family=None,
    ):
        """
        root_stick      : Stick
        stick_length    : float
        width, depth    : cross-section dimensions
        offset01        : attachment param along parent axis (0..1)
        initial_family  : "Y" or "Z" or None
                          If None → randomly choose.
        """
        self.sticks = [root_stick]

        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.width = width or Stick.DEFAULT_SIZE
        self.depth = depth or Stick.DEFAULT_SIZE
        self.offset01 = float(offset01)

        if initial_family is None:
            self.family = random.choice(["Y", "Z"])
        else:
            self.family = initial_family

    # =================================================================
    # INTERNAL CHILD CONSTRUCTION
    # =================================================================

    def _choose_face(self):
        """Given current family, choose one face."""
        if self.family == "Y":
            return random.choice(Y_FACES)
        else:
            return random.choice(Z_FACES)

    def _get_face_normal_and_offset(self, parent, face_index):
        """
        Returns:
            n         : unit face normal
            offset_d  : one-sided offset using PARENT dimension
        """
        f = parent.frame

        if face_index == 0:          # +Y
            n = f.yaxis.unitized()
            offset_d = parent.width   # FULL WIDTH (one sided)
        elif face_index == 2:        # -Y
            n = (-f.yaxis).unitized()
            offset_d = parent.width
        elif face_index == 1:        # +Z
            n = f.zaxis.unitized()
            offset_d = parent.depth   # FULL DEPTH (one sided)
        else:                        # -Z
            n = (-f.zaxis).unitized()
            offset_d = parent.depth

        return n, offset_d

    def _build_child(self, parent, face_index, stick_angle_deg):
        """
        Builds 1 new child using:
            - face_index
            - tangent + face normal blend
            - correct one-sided offset
            - COMPAS Box geometry
        """
        # 1) Parent local tangent
        f = parent.frame
        tangent = f.xaxis.unitized()

        # 2) Compute origin of child (offset from parent)
        n, offset_d = self._get_face_normal_and_offset(parent, face_index)

        t = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t)

        # one-sided offset into face
        child_center = axis_pt + n * offset_d

        # 3) Blend tangent + normal direction
        theta = math.radians(stick_angle_deg)
        d = tangent * math.cos(theta) + n * math.sin(theta)
        if d.length < 1e-6:
            d = tangent
        d.unitize()

        # 4) Build axis
        half_len = 0.5 * self.stick_length
        start = child_center - d * half_len
        end   = child_center + d * half_len
        axis  = Line(start, end)

        # 5) Create child stick (inherits parent frame orientation)
        child = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=f,
        )

        return child

    # =================================================================
    # Local alternation API (called externally from rf_core)
    # =================================================================

    def flip_family(self):
        """Switch Y ↔ Z when bridging is needed."""
        self.family = "Z" if self.family == "Y" else "Y"

    # =================================================================
    # PUBLIC L-system step
    # =================================================================

    def grow_once(self, stick_angle):
        """
        Produce one child based on current family.
        """
        parent = self.sticks[-1]

        # pick family face
        face_index = self._choose_face()

        # build child
        child = self._build_child(parent, face_index, stick_angle)

        self.sticks.append(child)
        return child

    def grow_chain(self, steps, stick_angle):
        """
        Generate multiple children.
        """
        result = []
        for _ in range(steps):
            result.append(self.grow_once(stick_angle))
        return result
