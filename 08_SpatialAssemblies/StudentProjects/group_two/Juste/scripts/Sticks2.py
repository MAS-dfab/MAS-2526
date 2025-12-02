import math
from compas.geometry import Line, Vector, Frame, Box, Transformation


def _stable_perpendicular(xaxis):
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(xaxis.dot(worldZ)) < 0.9 else worldY
    y = up.cross(xaxis)
    y.unitize()
    return y


class Stick:
    """Stick: axis (Line) + rectangular cross-section (width, depth)."""
    DEFAULT_LENGTH = 100.0
    SIZE = 5

    LENGTH = DEFAULT_LENGTH
    WIDTH = SIZE
    DEPTH = SIZE

    def __init__(self, axis, length = None, width=None, depth=None):
        if not isinstance(axis, Line):
            raise Exception("Stick axis must be a COMPAS Line.")

        self.axis = axis
        self.length = float(length) if length is not None else Stick.LENGTH
        self.width = float(width) if width is not None else Stick.WIDTH
        self.depth = float(depth) if depth is not None else Stick.DEPTH
        self.frame = self._compute_frame()

    def _compute_frame(self):
        x = self.axis.direction.copy()
        x.unitize()
        y = _stable_perpendicular(x)
        origin = self.axis.midpoint
        return Frame(origin, x, y)

    @property
    def geometry(self):
        length = self.axis.length
        base_box = Box(length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        base_box.transform(T)
        return base_box
