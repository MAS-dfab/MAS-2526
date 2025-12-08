# stick_fixed.py
from compas.geometry import Frame, Vector, Point, Line
from compas.geometry import Box


class Stick:
    DEFAULT_LEN = 1.0
    DEFAULT_SIZE = 0.05

    def __init__(self, axis: Line, length=None, width=None, depth=None, parent_frame=None):
        """
        axis : COMPAS Line
        """

        self.axis = axis
        self.length = length or Stick.DEFAULT_LEN
        self.width = width or Stick.DEFAULT_SIZE
        self.depth = depth or Stick.DEFAULT_SIZE

        # ---------------------------------------------------------
        # BUILD A PROPER FRAME (fix for the crash)
        # ---------------------------------------------------------

        origin = axis.start
        xaxis = Vector.from_start_end(axis.start, axis.end)

        if xaxis.length < 1e-6:
            xaxis = Vector(1, 0, 0)
        xaxis.unitize()

        # Build a stable y axis
        world_up = Vector(0, 0, 1)
        yaxis = world_up.cross(xaxis)
        if yaxis.length < 1e-6:
            yaxis = Vector(0, 1, 0)

        yaxis.unitize()

        # Correct COMPAS Frame
        self.frame = Frame(origin, xaxis, yaxis)

        # ---------------------------------------------------------
        # BOX GEOMETRY
        # ---------------------------------------------------------
        self.geometry = Box(self.frame, self.length, self.width, self.depth)

        # store useful info
        self.children = []
        self.parent_frame = self.frame

    # ---------------------------------------------------------
    # SIMPLE COLLISION TEST
    # ---------------------------------------------------------
    def aabb(self):
        """Axis-aligned bounding box for fast collision checks."""
        return self.geometry.aabb()

    def intersects(self, other, clearance=0.0):
        a = self.aabb()
        b = other.aabb()
        return a.intersects(b, tolerance=clearance)
