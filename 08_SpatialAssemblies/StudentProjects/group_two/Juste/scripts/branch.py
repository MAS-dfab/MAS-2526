# branch.py
# Stable L-system branching module for RootFrames.
# Produces consistent 3D growth aligned to parent stick frames.

from compas.geometry import Vector, Line
from stick_fixed import Stick


def face_direction(parent_frame, face_index):
    """
    Map a face index to a local normal + family code.

    Face mapping (local to parent frame):
        2,3 → Y-family (±yaxis)
        4,5 → Z-family (±zaxis)
    """

    if face_index in (2, 3):  # Y-family faces
        normal = parent_frame.yaxis.copy()
        fam = "Y"
    elif face_index in (4, 5):  # Z-family faces
        zaxis = parent_frame.zaxis
        normal = zaxis.copy()
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

    - root_stick   : initial Stick
    - stick_length : new branch length
    - width/depth  : new branch section size
    - offset01     : blend factor between tangent and normal directions
    - collision_clearance : extra radius tolerance for collision-safe mode
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

        If collision_safe is True and existing_sticks is provided,
        proposed children that collide are skipped (returns None).
        """

        parent = self.sticks[-1]
        pf = parent.frame

        # Determine which face to branch from
        normal, fam = face_direction(pf, face_index)
        if normal is None:
            return None

        # -------------------------------------------------
        # 1. Compute new origin ON THE PARENT FACE
        # -------------------------------------------------
        if fam == "Y":
            offset_dist = parent.width * 0.5 + self.width * 0.5
            offset_vec = pf.yaxis * offset_dist
        else:  # fam == "Z"
            offset_dist = parent.depth * 0.5 + self.depth * 0.5
            offset_vec = pf.zaxis * offset_dist

        start = pf.point + offset_vec

        # -------------------------------------------------
        # 2. Compute child direction (blend + rotate)
        # -------------------------------------------------
        x = pf.xaxis.copy()
        n = normal.copy()

        t = self.offset01
        dir_vec = (1 - t) * x + t * n
        if dir_vec.length < 1e-6:
            dir_vec = n
        dir_vec.unitize()

        ang = stick_angle * 3.141592653589793 / 180.0
        dir_vec = dir_vec.rotated(ang, x)

        # -------------------------------------------------
        # 3. Build axis
        # -------------------------------------------------
        end = start + dir_vec * self.stick_length
        axis = Line(start, end)

        # -------------------------------------------------
        # 4. Create child stick
        # -------------------------------------------------
        child = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=pf
        )
        child.family = fam
        child.parent = parent
        parent.children.append(child)

        # -------------------------------------------------
        # 5. Collision-safe gate (optional)
        # -------------------------------------------------
        if collision_safe and existing_sticks:
            for other in existing_sticks:
                if child.intersects(other, clearance=self.collision_clearance):
                    # Do not add this stick; treat as blocked
                    return None

        self.sticks.append(child)
        return child
