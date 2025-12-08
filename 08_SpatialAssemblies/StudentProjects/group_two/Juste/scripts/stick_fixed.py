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

        # -------------------------------------
        # 1. axis → COMPAS Line
        # -------------------------------------
        if isinstance(axis, Line):
            line = axis
        else:
            try:
                p0 = Point(axis.From.X, axis.From.Y, axis.From.Z)
                p1 = Point(axis.To.X, axis.To.Y, axis.To.Z)
                line = Line(p0, p1)
            except:
                raise ValueError("Stick(): axis is not a valid Line-like object.")

        self.axis = line
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width  = float(width  or Stick.DEFAULT_SIZE)
        self.depth  = float(depth  or Stick.DEFAULT_SIZE)

        # -------------------------------------
        # 2. Build oriented frame
        # -------------------------------------
        origin = line.point_at(0.5)
        xaxis = _unit(Vector.from_start_end(line.start, line.end), Vector(1,0,0))

        if parent_frame:
            z_parent = parent_frame.zaxis
            if abs(z_parent.dot(xaxis)) > 0.99:
                z_parent = _unit(parent_frame.yaxis.cross(xaxis), Vector(0,0,1))
            zaxis = _unit(z_parent, Vector(0,0,1))
            yaxis = _unit(zaxis.cross(xaxis), Vector(0,1,0))

        else:
            zaxis = Vector(0,0,1)
            if abs(zaxis.dot(xaxis)) > 0.99:
                zaxis = Vector(0,1,0)
            zaxis = _unit(zaxis, Vector(0,0,1))
            yaxis = _unit(zaxis.cross(xaxis), Vector(0,1,0))

        self.frame = Frame(origin, xaxis, yaxis)

        # -------------------------------------
        # 3. bookkeeping
        # -------------------------------------
        self.children = []
        self.parent_frame = self.frame
        self.family = None
        self.is_root = False
        self.is_bridge = False
        self.collided = False

    # -----------------------------------------
    def intersects(self, other, clearance=0.0):
        if not isinstance(other, Stick):
            return False
        r1 = 0.5 * max(self.width, self.depth) + clearance
        r2 = 0.5 * max(other.width, other.depth) + clearance
        try:
            d = self.axis.distance_to_line(other.axis)
        except:
            return False
        return d <= r1 + r2

    def __repr__(self):
        return f"Stick(len={self.length}, w={self.width}, d={self.depth}, fam={self.family})"
