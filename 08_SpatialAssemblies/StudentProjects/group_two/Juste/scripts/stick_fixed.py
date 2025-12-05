# stick_fixed.py
# Safe version for RhinoCode / Grasshopper

from compas.geometry import Line, Vector, Point, Frame, Box


# ------------------------------------------------------------
# Utility: stable perpendicular (used by bridge.py)
# ------------------------------------------------------------

def stable_perp(v):
    """Return a unit vector stably perpendicular to v."""
    if not isinstance(v, Vector):
        v = Vector(*v)

    if v.length < 1e-9:
        return Vector(1, 0, 0)

    v = v.unitized()

    ref = Vector(0, 0, 1)
    if abs(ref.dot(v)) > 0.9:
        ref = Vector(0, 1, 0)

    perp = ref.cross(v)
    if perp.length < 1e-9:
        return Vector(1, 0, 0)

    return perp.unitized()


# ------------------------------------------------------------
# Stick class (fully safe)
# ------------------------------------------------------------

class Stick:
    """
    A simple oriented stick represented as:
      - axis : compas Line
      - frame : compas Frame
      - geometry : compas Box
    """

    DEFAULT_LEN = 1.0
    DEFAULT_SIZE = 0.1

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):

        if not isinstance(axis, Line):
            raise TypeError("Stick axis must be a compas.geometry.Line.")

        # convert axis endpoints explicitly to COMPAS Points
        start = Point(axis.start.x, axis.start.y, axis.start.z)
        end   = Point(axis.end.x,   axis.end.y,   axis.end.z)
        self.axis = Line(start, end)

        self.length = float(length) if length else self.DEFAULT_LEN
        self.width  = float(width)  if width  else self.DEFAULT_SIZE
        self.depth  = float(depth)  if depth  else self.DEFAULT_SIZE

        # build stable 3D frame
        self.frame = self._make_frame(self.axis, parent_frame)

        # create oriented box geometry
        self.geometry = Box(self.frame, self.length, self.width, self.depth)


    # ------------------------------------------------------------
    # INTERNAL: build local frame from axis
    # ------------------------------------------------------------

    def _make_frame(self, axis, parent_frame):

        # --- X direction ---
        dx = axis.end.x - axis.start.x
        dy = axis.end.y - axis.start.y
        dz = axis.end.z - axis.start.z

        x = Vector(dx, dy, dz)
        if x.length < 1e-9:
            x = Vector(1, 0, 0)
        x.unitize()

        # --------------------------------------------------------
        # Y direction (preserve parent orientation if possible)
        # --------------------------------------------------------
        if parent_frame:
            py = Vector(parent_frame.yaxis.x,
                        parent_frame.yaxis.y,
                        parent_frame.yaxis.z)

            # projection of py onto plane perpendicular to x
            dot = py.dot(x)
            y = py - x * dot

            if y.length < 1e-6:
                pz = Vector(parent_frame.zaxis.x,
                            parent_frame.zaxis.y,
                            parent_frame.zaxis.z)
                y = pz - x * pz.dot(x)

            if y.length < 1e-6:
                y = Vector(0, 0, 1)
        else:
            # world fallback
            y = Vector(0, 0, 1)
            if abs(y.dot(x)) > 0.9:
                y = Vector(0, 1, 0)

        y.unitize()

        # --- Z direction ---
        z = x.cross(y)
        if z.length < 1e-6:
            z = Vector(0, 0, 1)
        z.unitize()

        # --------------------------------------------------------
        # Origin — COMPAS SAFE POINT (NEVER Rhino midpoint)
        # --------------------------------------------------------
        origin = axis.point_at(0.5)
        origin = Point(origin.x, origin.y, origin.z)

        # build frame
        return Frame(origin, x, y)


    # ------------------------------------------------------------
    # Collision helper (light)
    # ------------------------------------------------------------

    def intersects(self, other, clearance=0.0):
        """Capsule-like intersection test."""
        if not isinstance(other, Stick):
            return False

        r1 = 0.5 * max(self.width, self.depth) + clearance
        r2 = 0.5 * max(other.width, other.depth) + clearance
        R  = r1 + r2

        try:
            d = self.axis.distance_to_line(other.axis)
        except Exception:
            return False

        return d <= R
