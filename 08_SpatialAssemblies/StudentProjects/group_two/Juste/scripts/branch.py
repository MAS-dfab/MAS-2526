# ============================================================
# branch.py — Final L-system branching module
# ============================================================

from compas.geometry import Point, Vector, Line, Frame
from stick_fixed import Stick

# Face → direction mapping (local to parent frame)
# 2,3 = Y-family
# 4,5 = Z-family
FAMILY_MAP = {
    2: "Y",
    3: "Y",
    4: "Z",
    5: "Z",
}


def face_direction(parent_frame, face_index):
    """Returns direction + offset normal based on face index."""

    if face_index in (2, 3):  # Y-family
        normal = parent_frame.yaxis
        fam = "Y"
    elif face_index in (4, 5):  # Z-family
        normal = parent_frame.zaxis
        fam = "Z"
    else:
        # invalid face = ignore
        return None, None

    # reverse if odd face index to alternate direction
    if face_index % 2 == 1:
        normal = -normal

    return normal, fam


class BranchingModule:

    def __init__(self, root_stick, stick_length, width, depth, offset01=0.5):
        self.root = root_stick
        self.stick_length = stick_length
        self.width = width
        self.depth = depth
        self.offset01 = offset01

        # collect all sticks grown from this root
        self.sticks = [root_stick]


    # ----------------------------------------------------------------------
    def _axis_for_child(self, parent_stick, normal_vec, angle_deg):
        """Builds a new axis using:

        axis_dir = blend( parent.xaxis , normal ) rotated by angle.
        """

        pf = parent_stick.frame
        x = pf.xaxis.copy()
        n = normal_vec.copy()

        # blend weight from slider
        t = self.offset01
        dir_vec = (1 - t) * x + t * n
        if dir_vec.length < 1e-6:
            dir_vec = n.copy()
        dir_vec.unitize()

        # rotate around parent's X-axis
        ang = angle_deg * 3.1415926 / 180.0
        dir_vec.rotate(ang, x)

        start = pf.point
        end = start + dir_vec * self.stick_length
        return Line(start, end)


    # ----------------------------------------------------------------------
    def grow_once(self, face_index, stick_angle):
        """One L-system expansion step."""

        parent = self.sticks[-1]
        normal, fam = face_direction(parent.frame, face_index)

        if normal is None:
            return None  # invalid face

        # determine full offset distance
        offset_dist = self.width if fam == "Y" else self.depth

        # offset start point
        start = parent.frame.point + normal * offset_dist

        # compute axis using tangent+normal blending
        axis = self._axis_for_child(parent, normal, stick_angle)

        # shift axis origin
        axis = Line(start, start + (axis.end - axis.start))

        # create new stick
        child = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=parent.frame
        )

        child.family = fam  # store family on new stick

        self.sticks.append(child)
        return child
