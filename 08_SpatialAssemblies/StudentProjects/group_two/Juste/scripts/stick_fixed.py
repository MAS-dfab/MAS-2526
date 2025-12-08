# stick_fixed.py
# Minimal, robust Stick object for RootFrames / Branching / Bridging.
# Geometry (Brep) is built in GH from:
#   - self.axis  (compas Line)
#   - self.frame (compas Frame)
#   - self.length, self.width, self.depth

from compas.geometry import Point, Vector, Line, Frame, distance_line_line

EPS = 1e-9


def _safe_unit(vec, fallback):
    """Return a unit copy of vec; if degenerate, use fallback."""
    v = vec.copy()
    if v.length < EPS:
        v = fallback.copy()
    v.unitize()
    return v


class Stick:
    DEFAULT_LEN = 250.0
    DEFAULT_SIZE = 13.0

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):
        """
        axis          : COMPAS Line or RhinoCommon Line-like
        length/width/depth : dimensions along local x/y/z
        parent_frame  : optional COMPAS Frame (for orientation continuity)
        """

        # -------------------------------------------------
        # 1. Sanitize axis → COMPAS Line
        # -------------------------------------------------
        if isinstance(axis, Line):
            line = axis
        else:
            # RhinoCommon Line-like
            try:
                p0 = Point(axis.From.X, axis.From.Y, axis.From.Z)
                p1 = Point(axis.To.X, axis.To.Y, axis.To.Z)
                line = Line(p0, p1)
            except Exception:
                raise ValueError("Stick(): axis is not a valid Line-like object.")

        self.axis = line
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width = float(width or Stick.DEFAULT_SIZE)
        self.depth = float(depth or Stick.DEFAULT_SIZE)

        self.parent = None
        self.children = []
        self.family = None        # "Y", "Z", "BRIDGE"
        self.is_root = False
        self.is_bridge = False
        self.collided = False

        # -------------------------------------------------
        # 2. Robust frame construction
        # -------------------------------------------------
        # Origin at axis MIDPOINT (better for offsets)
        origin = line.point_at(0.5)

        # Local x-axis = axis direction
        x = Vector.from_start_end(line.start, line.end)
        x = _safe_unit(x, Vector(1, 0, 0))

        # We want to inherit parent orientation if present,
        # but keep the frame orthonormal and 3D.
        if parent_frame is not None:
            # Use parent's y-axis, but make it perpendicular to x
            py = parent_frame.yaxis
            py_proj = py - x * py.dot(x)
            y = _safe_unit(py_proj, Vector(0, 0, 1))

            # z = x × y
            z = x.cross(y)
            z = _safe_unit(z, Vector(0, 0, 1))
        else:
            # No parent → pick any stable perpendicular basis
            trial = Vector(0, 0, 1)
            if abs(trial.dot(x)) > 0.99:
                trial = Vector(0, 1, 0)
            y = _safe_unit(trial.cross(x), Vector(0, 1, 0))
            z = x.cross(y)
            z = _safe_unit(z, Vector(0, 0, 1))

        self.frame = Frame(origin, x, y)
        # keep z for debugging if needed
        self._zaxis = z

    # ---------------------------------------------------------
    # Collision helper — approximate capsule/cylinder distance
    # ---------------------------------------------------------

    def intersects(self, other, clearance=0.0):
        if not isinstance(other, Stick):
            return False

        # use distance between line segments
        d = distance_line_line((self.axis.start, self.axis.end),
                               (other.axis.start, other.axis.end))

        r1 = 0.5 * max(self.width, self.depth)
        r2 = 0.5 * max(other.width, other.depth)
        return d <= (r1 + r2 + clearance)

    def __repr__(self):
        return "Stick(len={:.3f}, w={:.3f}, d={:.3f}, fam={})".format(
            self.length, self.width, self.depth, self.family
        )
