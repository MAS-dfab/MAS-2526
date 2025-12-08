# branch.py
# Stable L-system branching module for RootFrames.
# Branches:
#   - start on a parent Y or Z face
#   - direction is blend(parent.xaxis, face_normal)
#   - rotation is around parent.xaxis (Option B)
#
# No world-XYZ except for degenerate fallbacks.


# stick_fixed.py
from compas.geometry import Point, Vector, Line, Frame

EPS = 1e-9

def _unit(vec, fallback):
    v = vec.copy()
    if v.length < EPS:
        v = fallback.copy()
    v.unitize()
    return v


class Stick:
    DEFAULT_LEN = 250.0
    DEFAULT_SIZE = 13.0

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):
        ...
        # full Stick definition here



def face_direction(parent_frame, face_index):
    """
    Map a face index to a local normal + family code.

    Convention:
        2,3 -> Y-family (± yaxis)
        4,5 -> Z-family (± zaxis)
        All others: invalid (no branch).
    """

    if face_index in (2, 3):  # Y-family faces
        normal = parent_frame.yaxis.copy()
        fam = "Y"

    elif face_index in (4, 5):  # Z-family faces
        # z-axis from frame; guaranteed orthogonal in our Stick class.
        normal = parent_frame.zaxis.copy()
        fam = "Z"

    else:
        return None, None

    # reverse if odd face index
    if face_index % 2 == 1:
        normal *= -1.0

    return normal, fam


class BranchingModule:
    """
    Branching L-system for a single root stick.

    Parameters:
        root_stick          : initial Stick
        stick_length        : child stick length
        width, depth        : child stick cross-section
        offset01            : blend factor between tangent and normal
        collision_clearance : radius for collision-safe growth
    """

    def __init__(self, root_stick, stick_length, width, depth,
                 offset01=0.5, collision_clearance=0.0):

        self.root = root_stick
        self.stick_length = stick_length
        self.width = width
        self.depth = depth
        self.offset01 = offset01
        self.collision_clearance = collision_clearance

        self.sticks = [root_stick]

    # ---------------------------------------------------------
    # BRANCHING STEP
    # ---------------------------------------------------------

    def grow_once(self, face_index, stick_angle,
                  existing_sticks=None, collision_safe=False):
        """
        Single branching step.

        - face_index indicates which parent face to branch from.
        - stick_angle is in degrees.
        - if collision_safe is True, the candidate child is tested
          against existing_sticks and skipped if it collides.
        """

        parent = self.sticks[-1]
        pf = parent.frame

        normal, fam = face_direction(pf, face_index)
        if normal is None:
            return None

        x = pf.xaxis.copy()
        y = pf.yaxis.copy()
        z = pf.zaxis.copy()

        # -------------------------------------------------
        # 1. Compute new origin ON THE PARENT FACE
        # -------------------------------------------------
        if fam == "Y":
            offset_dist = parent.width * 0.5 + self.width * 0.5
            offset_dir = y
        else:  # "Z"
            offset_dist = parent.depth * 0.5 + self.depth * 0.5
            offset_dir = z

        start = pf.point + offset_dir * offset_dist

        # -------------------------------------------------
        # 2. Compute child direction (blend + rotate around local X)
        # -------------------------------------------------
        t = self.offset01
        dir_vec = (1 - t) * x + t * normal
        if dir_vec.length < EPS:
            dir_vec = normal
        dir_vec.unitize()

        ang = stick_angle * 3.141592653589793 / 180.0
        dir_vec = dir_vec.rotated(ang, x)   # Option B: rotate around parent.xaxis

        end = start + dir_vec * self.stick_length
        axis = Line(start, end)

        # -------------------------------------------------
        # 3. Build child stick
        # -------------------------------------------------
        child = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=pf,
        )
        child.family = fam
        child.parent = parent
        parent.children.append(child)

        # -------------------------------------------------
        # 4. Collision-safe gate
        # -------------------------------------------------
        if collision_safe and existing_sticks:
            for other in existing_sticks:
                if child.intersects(other, clearance=self.collision_clearance):
                    # discard this child
                    return None

        self.sticks.append(child)
        return child
