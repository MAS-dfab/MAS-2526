# stick_fixed.py
from compas.geometry import Point, Vector, Line, Frame

EPS = 1e-9

def _safe_unit(v, fallback):
    v = v.copy()
    if v.length < EPS:
        v = fallback.copy()
    v.unitize()
    return v


class Stick:
    DEFAULT_LEN = 250.0
    DEFAULT_SIZE = 13.0

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):
        self.axis = axis
        self.length = length or Stick.DEFAULT_LEN
        self.width  = width  or Stick.DEFAULT_SIZE
        self.depth  = depth  or Stick.DEFAULT_SIZE

        self.parent = None
        self.children = []
        self.family = None
        self.is_root = False
        self.is_bridge = False
        self.collided = False

        # -----------------------------
        # FRAME CONSTRUCTION (CRITICAL)
        # -----------------------------
        p0 = axis.start
        p1 = axis.end

        x = Vector.from_start_end(p0, p1)
        x = _safe_unit(x, Vector(1, 0, 0))

        # Build y-axis from parent if available
        if parent_frame:
            # Project parent.y onto plane perpendicular to x
            parent_y = parent_frame.yaxis.copy()
            parent_y -= x * parent_y.dot(x)
            y = _safe_unit(parent_y, Vector(0, 0, 1))
        else:
            # If no parent, use generic perpendicular to x
            trial = Vector(0, 0, 1)
            if abs(trial.dot(x)) > 0.99:
                trial = Vector(0, 1, 0)
            y = _safe_unit(trial.cross(x), Vector(0,1,0))

        z = x.cross(y)
        z = _safe_unit(z, Vector(0,0,1))

        self.frame = Frame(p0, x, y)
        self.frame._zaxis = z.copy()   # store for debugging

    # --------------------------------------------------------
    # COLLISION CHECK (bounding cylinder approximation)
    # --------------------------------------------------------

    def intersects(self, other, clearance=0.0):
        a0 = self.axis.start
        a1 = self.axis.end
        b0 = other.axis.start
        b1 = other.axis.end

        # distance between two segments
        import compas.geometry as cg
        d = cg.distance_line_line((a0, a1), (b0, b1))

        thresh = (self.width + other.width) * 0.5 + clearance
        return d < thresh
