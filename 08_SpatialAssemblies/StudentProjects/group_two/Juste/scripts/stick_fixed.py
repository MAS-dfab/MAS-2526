# stick_fixed.py
# COMPAS-only stick with manual box geometry

from compas.geometry import Point, Vector, Line, Frame

def build_frame_from_axis(axis, up_hint=None):
    start = axis.start
    end   = axis.end

    x = Vector.from_start_end(start, end)
    if x.length < 1e-6:
        x = Vector(1, 0, 0)
    x.unitize()

    if up_hint is None:
        up_hint = Vector(0, 0, 1)

    y = up_hint.cross(x)
    if y.length < 1e-6:
        y = Vector(0, 1, 0)
    y.unitize()

    origin = axis.point_at(0.5)
    return Frame(origin, x, y)


class Stick:
    DEFAULT_LEN  = 1.0
    DEFAULT_SIZE = 0.2

    def __init__(self, axis: Line, length=None, width=None, depth=None, parent_frame=None):

        self.axis   = axis
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width  = float(width  or Stick.DEFAULT_SIZE)
        self.depth  = float(depth  or Stick.DEFAULT_SIZE)

        # orientation
        if parent_frame:
            o = axis.point_at(0.5)
            x = parent_frame.xaxis
            y = parent_frame.yaxis
            self.frame = Frame(o, x, y)
        else:
            self.frame = build_frame_from_axis(axis)

        # ------------------------------------------------------
        # BUILD MANUAL BOX GEOMETRY (no compas Box object)
        # ------------------------------------------------------
        f = self.frame
        cx = 0.5 * self.length
        cy = 0.5 * self.width
        cz = 0.5 * self.depth

        # 8 vertices of the box
        verts = []
        for dx in [-cx, cx]:
            for dy in [-cy, cy]:
                for dz in [-cz, cz]:
                    p = f.point + f.xaxis * dx + f.yaxis * dy + f.zaxis * dz
                    verts.append(Point(p.x, p.y, p.z))

        self.geometry = {"vertices": verts}

    # collision capsule approximation
    def intersects(self, other, clearance=0.0):
        r1 = 0.5 * max(self.width, self.depth)
        r2 = 0.5 * max(other.width, other.depth)
        R  = r1 + r2 + clearance
        try:
            d = self.axis.distance_to_line(other.axis)
        except:
            return False
        return d <= R

    def __repr__(self):
        return f"Stick(len={self.length}, w={self.width}, d={self.depth})"
