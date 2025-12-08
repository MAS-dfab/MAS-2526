# branch.py
# Stable L-system branching module for RootFrames.
# Produces consistent 3D growth aligned to parent stick frames.

from compas.geometry import Point, Vector, Line
from stick_fixed import Stick

# Family mapping (Y and Z based on parent frame axes)
FAMILY_MAP = {2: "Y", 3: "Y", 4: "Z", 5: "Z"}


def face_direction(parent_frame, face_index):
    """
    Return (normal_vector, family_code).
    normal_vector is guaranteed to be a *copy* and safe for mutation.
    """

    if face_index in (2, 3):         # Y-family
        normal = parent_frame.yaxis.copy()
        fam = "Y"

    elif face_index in (4, 5):       # Z-family
        # parent z-axis = cross(x,y). Always valid in our patched system.
        zaxis = parent_frame.zaxis if hasattr(parent_frame, "zaxis") else parent_frame.xaxis.cross(parent_frame.yaxis)
        normal = zaxis.copy()
        fam = "Z"

    else:
        return None, None

    if face_index % 2 == 1:
        normal *= -1.0

    return normal, fam


class BranchingModule:

    def __init__(self, root_stick, stick_length, width, depth, offset01=0.5):
        self.root = root_stick
        self.stick_length = stick_length
        self.width = width
        self.depth = depth
        self.offset01 = offset01

        self.sticks = [root_stick]

    # ---------------------------------------------------------
    # CHILD AXIS GENERATION
    # ---------------------------------------------------------

    def _axis_for_child(self, parent_stick, normal_vec, angle_deg):
        """
        Produces a new axis direction based on:
        - parent frame x-axis (main direction)
        - blend with local normal (Y or Z)
        - rotation around local x-axis
        """

        pf = parent_stick.frame
        x = pf.xaxis.copy()
        n = normal_vec.copy()

        # Blend
        t = self.offset01
        dir_vec = (1 - t) * x + t * n
        if dir_vec.length < 1e-9:
            dir_vec = n
        dir_vec.unitize()

        # Rotate in parent stick coordinates
        ang = angle_deg * 3.14159265359 / 180.0
        rotated = dir_vec.rotated(ang, x)

        start = pf.point
        end = start + rotated * self.stick_length
        return Line(start, end)

    # ---------------------------------------------------------
    # BRANCHING STEP
    # ---------------------------------------------------------

    def grow_once(self, face_index, stick_angle):

        parent = self.sticks[-1]
        normal, fam = face_direction(parent.frame, face_index)

        if normal is None:
            return None

        # Compute offset distance
        offset_dist = self.width if fam == "Y" else self.depth

        # Offset starting point along local Y/Z family direction
        start = parent.frame.point + normal * offset_dist

        # Compute axis direction
        axis = self._axis_for_child(parent, normal, stick_angle)

        # Move axis to offset start
        axis = Line(start, start + (axis.end - axis.start))

        # Create new stick
        child = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=parent.frame
        )

        child.family = fam
        child.parent = parent
        parent.children.append(child)

        self.sticks.append(child)
        return child
