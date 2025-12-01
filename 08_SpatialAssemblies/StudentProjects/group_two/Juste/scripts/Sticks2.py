from compas.geometry import Point, Vector, Frame, Line, Box

import math


class Stick:
    def __init__(self, axis, width, depth):
        self.axis = axis          # COMPAS Line
        self.width = width        # Y size
        self.depth = depth        # Z size
        self.frame = self._build_frame()   # COMPAS Frame

    def _get_stick_frame(self):
        # X-axis = stick direction
        x = self.axis.direction.copy()
        x.unitize()

        # Choose a stable up vector
        worldZ = Vector(0,0,1)
        worldY = Vector(0,1,0)
        up = worldZ if abs(x.dot(worldZ)) < 0.9 else worldY

        # Build Y-axis
        y = up.cross(x)
        y.unitize()

        # Build Z-axis *explicitly*
        z = x.cross(y)
        z.unitize()

        # Build full right-handed Frame
        return Frame(self.axis.midpoint, x, y)


    @property
    def geometry(self):
        """Return a COMPAS Box aligned with this stick."""
        frame_tuple = (self.frame.point, self.frame.xaxis, self.frame.yaxis)
        # Your COMPAS build: Box(xsize, ysize, zsize, frame_tuple)
        return Box(self.axis.length, self.width, self.depth, Frame(self.axis.midpoint, x, y))


