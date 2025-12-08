from compas.geometry import Frame, Vector, Point, Line
from compas.geometry import Box


class Stick:
    DEFAULT_LEN = 1.0
    DEFAULT_SIZE = 0.05

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):
        """
        axis : COMPAS Line or RhinoCommon Line or anything vaguely line-like.
        """

        # --------------------------------------------
        # SANITIZE AXIS — convert ANYTHING into COMPAS Line
        # --------------------------------------------
        if isinstance(axis, Line):
            line = axis
        else:
            try:
                # RhinoCommon Line: axis.From, axis.To
                p0 = Point(axis.From.X, axis.From.Y, axis.From.Z)
                p1 = Point(axis.To.X, axis.To.Y, axis.To.Z)
                line = Line(p0, p1)
            except:
                raise ValueError("Stick(): axis is not a valid Line-like object.")

        self.axis = line
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width = float(width or Stick.DEFAULT_SIZE)
        self.depth = float(depth or Stick.DEFAULT_SIZE)

        # --------------------------------------------
        # ALWAYS REBUILD A VALID COMPAS FRAME
        # --------------------------------------------

        origin = line.start
        xaxis = Vector.from_start_end(line.start, line.end)

        if xaxis.length < 1e-6:
            xaxis = Vector(1, 0, 0)
        xaxis.unitize()

        world_up = Vector(0, 0, 1)
        yaxis = world_up.cross(xaxis)
        if yaxis.length < 1e-6:
            yaxis = Vector(0, 1, 0)
        yaxis.unitize()

        self.frame = Frame(origin, xaxis, yaxis)

        # --------------------------------------------
        # BUILD BOX GEOMETRY
        # --------------------------------------------
        self.geometry = Box(self.frame, self.length, self.width, self.depth)

        self.children = []
        self.parent_frame = self.frame

    # -----------------------
    # COLLISION UTILITIES
    # -----------------------

    def aabb(self):
        return self.geometry.aabb()

    def intersects(self, other, clearance=0.0):
        a = self.aabb()
        b = other.aabb()
        return a.intersects(b, tolerance=clearance)
