# stick_fixed.py
from compas.geometry import Point, Vector, Line, Frame

EPS = 1e-9

def _unit(v, fallback=None):
    if v.length < EPS:
        return fallback.copy() if fallback else v
    u = v.copy()
    u.unitize()
    return u


class Stick:
    """
    A 3D stick defined ONLY by:
      - an axis (compas Line)
      - width/depth
      - a parent frame (for child orientation logic)

    No world-XYZ assumptions. No overrides.
    """

    DEFAULT_LEN = 250.0
    DEFAULT_SIZE = 13.0

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):
        self.axis = axis
        self.length = length or self.DEFAULT_LEN
        self.width = width or self.DEFAULT_SIZE
        self.depth = depth or self.DEFAULT_SIZE

        # parent relationship
        self.parent = None
        self.children = []
        self.family = None   # "Y", "Z", or "BRIDGE"
        self.is_root = False
        self.is_bridge = False
        self.collided = False

        # ===============================
        # BUILD THIS STICK'S LOCAL FRAME
        # ===============================
        x = Vector.from_start_end(axis.start, axis.end)
        x = _unit(x, Vector(1,0,0))

        # derive y and z from parent frame **without** world-axis re-projection
        if parent_frame:
            # preserve parent orientation
            # but ensure orthogonality
            y = parent_frame.yaxis - x * parent_frame.yaxis.dot(x)
            if y.length < EPS:
                # fallback: rotate parent's z instead
                y = parent_frame.zaxis - x * parent_frame.zaxis.dot(x)
            y = _unit(y, Vector(0,1,0))

            z = x.cross(y)
            if z.length < EPS:
                z = Vector(0,0,1)
            z.unitize()

        else:
            # ROOT STICK INITIALIZATION
            # fallback stable coordinate system
            fallback_y = Vector(0,0,1) if abs(x.dot(Vector(0,0,1))) < 0.9 else Vector(0,1,0)
            y = x.cross(fallback_y)
            y = _unit(y, Vector(0,1,0))
            z = x.cross(y)
            z.unitize()

        self.frame = Frame(axis.start, x, y)
        self.frame._zaxis = z   # compas stores z internally, expose it anyway

    # --------------------------------------------------------
    # COLLISION CHECK
    # --------------------------------------------------------
    def intersects(self, other, clearance=0.0):
        """
        True if cylindrical approximation intersects.
        Uses line-line distance.
        """

        p0 = self.axis.start
        p1 = self.axis.end
        p2 = other.axis.start
        p3 = other.axis.end

        u = Vector.from_start_end(p0, p1)
        v = Vector.from_start_end(p2, p3)
        w0 = Vector.from_start_end(p2, p0)

        a = u.dot(u)
        b = u.dot(v)
        c = v.dot(v)
        d = u.dot(w0)
        e = v.dot(w0)

        denom = a * c - b * b
        sc = 0.0
        tc = 0.0

        if denom < EPS:
            # nearly parallel
            sc = 0.0
            tc = (b > c and e/b) or (e/c)
        else:
            sc = (b * e - c * d) / denom
            tc = (a * e - b * d) / denom

        sc = max(0, min(1, sc))
        tc = max(0, min(1, tc))

        psc = p0 + u * sc
        ptc = p2 + v * tc

        dist = psc.distance_to_point(ptc)
        threshold = (self.width + other.width) * 0.5 + clearance

        return dist < threshold
